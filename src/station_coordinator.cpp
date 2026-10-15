#include "station_coordinator.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace libpyelite {
namespace {
using Clock = std::chrono::steady_clock;
double now() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
}  // namespace

StationCoordinator::StationCoordinator(std::shared_ptr<StationGuard> guard, NativeDriver &left, NativeDriver &right,
                                       std::shared_ptr<ELITE::RtsiIOInterface> left_rtsi,
                                       std::shared_ptr<ELITE::RtsiIOInterface> right_rtsi)
    : guard_(std::move(guard)), drivers_{{&left, &right}}, rtsi_{{std::move(left_rtsi), std::move(right_rtsi)}},
      period_(left.config_.servoj_time) {
  if(!guard_ || !rtsi_[0] || !rtsi_[1]) throw std::invalid_argument("StationCoordinator needs a guard and both RTSI feeds");
  if(&left == &right) throw std::invalid_argument("StationCoordinator needs two distinct arms");
  if(std::abs(left.config_.servoj_time - right.config_.servoj_time) > 1e-12)
    throw std::invalid_argument("Coordinated arms need the same servoj_time");
  if(!left.externalPlanning() || !right.externalPlanning())
    throw std::logic_error("Set both drivers to external planning before coordinating them");
}

StationCoordinator::~StationCoordinator() { stop(); }

void StationCoordinator::start() {
  if(running_) return;
  if(thread_.joinable()) thread_.join();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_ = CoordinatorStatus{};
  }
  guard_->reset();
  running_ = true;
  thread_ = std::thread(&StationCoordinator::loop, this);
}

void StationCoordinator::stop() {
  running_ = false;
  if(thread_.joinable()) thread_.join();
}

CoordinatorStatus StationCoordinator::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = status_;
  result.running = running_;
  return result;
}

void StationCoordinator::failBoth(const std::string &message) noexcept {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.error = message;
  }
  running_ = false;
  for(auto *driver : drivers_) driver->fail("Station collision layer: " + message);
}

void StationCoordinator::loop() noexcept {
  auto next = Clock::now();
  std::array<std::uint64_t, 2> consumed{};
  std::array<bool, 2> was_running{};
  try {
    while(running_) {
      const double begin = now();
      // A driver that faulted (feedback, safety mode, socket) stops its partner too.
      for(std::size_t a = 0; a < 2; ++a) {
        const auto s = drivers_[a]->status();
        if(!s.error.empty()) {
          failBoth(std::string(a == 0 ? "left" : "right") + " arm fault: " + s.error);
          return;
        }
      }
      // An arm being started or stopped (lifecycle lock held) or not running is
      // a fixed obstacle this period; holding the lock keeps it from stopping mid-step.
      std::array<std::unique_lock<std::mutex>, 2> locks;
      std::array<Planner *, 2> planners{};
      std::array<NativeDriver::Target, 2> targets{};
      std::array<PinkArmVector, 2> fixed{};
      for(std::size_t a = 0; a < 2; ++a) {
        auto *driver = drivers_[a];
        locks[a] = std::unique_lock<std::mutex>(driver->lifecycle_mutex_, std::try_to_lock);
        const bool active = locks[a].owns_lock() && driver->running_;
        if(active && !was_running[a]) consumed[a] = 0;
        was_running[a] = active;
        if(active) {
          driver->consumeTarget(consumed[a], targets[a]);
          planners[a] = &driver->planner_;
        } else {
          if(locks[a].owns_lock()) locks[a].unlock();
          fixed[a] = rtsi_[a]->getActualJointPositions();
        }
      }
      const auto result = guard_->tick(planners[0], planners[1], fixed[0], fixed[1], begin);
      for(std::size_t a = 0; a < 2; ++a)
        if(planners[a]) drivers_[a]->publish(targets[a], begin);
      const double elapsed = now() - begin;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.last = result;
        ++status_.ticks;
        if(elapsed > period_) ++status_.overruns;
        status_.max_tick_s = std::max(status_.max_tick_s, elapsed);
      }
      for(auto &lock : locks)
        if(lock.owns_lock()) lock.unlock();
      next += std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(period_));
      if(next < Clock::now()) next = Clock::now();
      std::this_thread::sleep_until(next);
    }
  } catch(const std::exception &e) {
    failBoth(e.what());
  } catch(...) {
    failBoth("unknown failure");
  }
}
}  // namespace libpyelite
