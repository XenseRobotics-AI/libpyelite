#pragma once
#include "native_controller.hpp"
#include "station_guard.hpp"

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace libpyelite {

struct CoordinatorStatus {
  bool running{false};
  std::string error;
  GuardResult last;
  std::uint64_t ticks{0}, overruns{0};
  double max_tick_s{0};
};

// Hardware loop of the station collision layer: one planning thread for both
// arms, replacing each NativeDriver's own planner thread (setExternalPlanning).
// Every period it consumes each running arm's latest target, runs
// StationGuard::tick on both planners and publishes the result to each arm's
// sender. An arm whose servo is stopped (or being started/stopped, e.g. MoveJ)
// is a fixed obstacle at its measured joints. Any failure, on either arm or in
// the guard, stops both arms' senders with the error latched.
class StationCoordinator {
 public:
  StationCoordinator(std::shared_ptr<StationGuard> guard, NativeDriver &left, NativeDriver &right,
                     std::shared_ptr<ELITE::RtsiIOInterface> left_rtsi,
                     std::shared_ptr<ELITE::RtsiIOInterface> right_rtsi);
  ~StationCoordinator();
  StationCoordinator(const StationCoordinator &) = delete;
  StationCoordinator &operator=(const StationCoordinator &) = delete;

  void start();
  void stop();
  CoordinatorStatus status() const;

 private:
  void loop() noexcept;
  void failBoth(const std::string &message) noexcept;

  std::shared_ptr<StationGuard> guard_;
  std::array<NativeDriver *, 2> drivers_;
  std::array<std::shared_ptr<ELITE::RtsiIOInterface>, 2> rtsi_;
  double period_;
  std::atomic<bool> running_{false};
  std::thread thread_;
  mutable std::mutex mutex_;
  CoordinatorStatus status_;
};
}  // namespace libpyelite
