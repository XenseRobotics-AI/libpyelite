#include "native_controller.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace libpyelite {
namespace {
using Clock=std::chrono::steady_clock;
double now() { return std::chrono::duration<double>(Clock::now().time_since_epoch()).count(); }
PlannerConfig plannerConfig(const DriverConfig &c) {
  auto p=c.planner; p.period=c.servoj_time; return p;
}
void waitTick(Clock::time_point &next,double period) {
  next+=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(period));
  if(next<Clock::now()) next=Clock::now();
  std::this_thread::sleep_until(next);
}
}
NativeDriver::NativeDriver(DriverConfig config)
 : config_(std::move(config)),planner_(config_.model_path,config_.tool,config_.ik,plannerConfig(config_)) {
  for(double v:{config_.feedback_timeout,config_.planner_timeout,
      config_.fk_position_tolerance,config_.fk_orientation_tolerance})
    if(!std::isfinite(v)||v<=0) throw std::invalid_argument("Invalid native timeout or FK tolerance");
  if(config_.command_timeout_ms<=0) throw std::invalid_argument("Invalid command timeout");
  driver_=std::make_unique<ELITE::EliteDriver>(config_);
}
NativeDriver::~NativeDriver() {
  stopServo();
  try { driver_->stopControl(200); } catch(...) {}
}
bool NativeDriver::isRobotConnected() { return driver_->isRobotConnected(); }
bool NativeDriver::sendExternalControlScript() { return driver_->sendExternalControlScript(); }
bool NativeDriver::setPayload(double mass, const ELITE::vector3d_t &cog) {
  return driver_->setPayload(mass,cog);
}
bool NativeDriver::getPrimaryPackage(std::shared_ptr<ELITE::PrimaryPackage> package, int timeout_ms) {
  return driver_->getPrimaryPackage(std::move(package),timeout_ms);
}
bool NativeDriver::writeIdle(int ms) {
  if(running_) throw std::logic_error("Native sender owns the reverse socket");
  return driver_->writeIdle(ms);
}
void NativeDriver::startServo(std::shared_ptr<ELITE::RtsiIOInterface> rtsi) {
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
  if(running_) return;
  if(planner_thread_.joinable()) planner_thread_.join();
  if(sender_thread_.joinable()) sender_thread_.join();
  if(!rtsi || !rtsi->isConnected() || !driver_->isRobotConnected())
    throw std::runtime_error("Cannot start servo without feedback and external control");
  rtsi_=std::move(rtsi);
  // Wait for an advancing controller timestamp; cached/stale/zero feedback is not a seed.
  const double initial=rtsi_->getTimestamp(),deadline=now()+config_.feedback_timeout;
  double stamp=initial;
  while(stamp==initial && now()<deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); stamp=rtsi_->getTimestamp();
  }
  if(!std::isfinite(stamp)||stamp<=0||stamp==initial)
    throw std::runtime_error("RTSI timestamp is not advancing");
  PinkArmVector q{},tcp{};
  bool coherent=false;
  for(int retry=0;retry<20;++retry) {
    double before=rtsi_->getTimestamp();
    q=rtsi_->getActualJointPositions(); tcp=rtsi_->getActualTCPPose();
    if(before==rtsi_->getTimestamp()) { coherent=true; break; }
  }
  if(!coherent) throw std::runtime_error("Could not obtain a coherent RTSI seed");
  planner_.seed(q);
  const auto fk=planner_.pose(),actual=poseFromControllerTcp(tcp);
  if(!validPose(actual)) throw std::runtime_error("Invalid measured TCP");
  using M=Eigen::Matrix<double,4,4,Eigen::RowMajor>;
  const Eigen::Map<const M> a(fk.data()),b(actual.data());
  double position=(a.block<3,1>(0,3)-b.block<3,1>(0,3)).norm();
  double angle=Eigen::Quaterniond(a.block<3,3>(0,0)).angularDistance(Eigen::Quaterniond(b.block<3,3>(0,0)));
  if(position>config_.fk_position_tolerance || angle>config_.fk_orientation_tolerance)
    throw std::runtime_error("Model FK does not match controller TCP: position="+
      std::to_string(position)+" m, orientation="+std::to_string(angle)+
      " rad. Verify URDF/base/tool calibration; no servo targets sent.");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    target_=Target{}; target_.sequence=++generation_; target_.cancel=true;
    status_=NativeStatus{}; status_.commanded_q=q; status_.commanded_pose=fk;
    command_time_=now(); running_=true;
  }
  try {
    planner_thread_=std::thread(&NativeDriver::planLoop,this);
    sender_thread_=std::thread(&NativeDriver::sendLoop,this);
  } catch(...) {
    running_=false;
    if(planner_thread_.joinable()) planner_thread_.join();
    throw;
  }
}
void NativeDriver::stopServo() {
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
  running_=false;
  if(planner_thread_.joinable()) planner_thread_.join();
  if(sender_thread_.joinable()) sender_thread_.join();
}
void NativeDriver::stopControl(int ms) { stopServo(); driver_->stopControl(ms); }
void NativeDriver::fail(const std::string &message) noexcept {
  std::lock_guard<std::mutex> lock(mutex_);
  status_.error=message; status_.reset_active=false; running_=false;
}
NativeStatus NativeDriver::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result=status_; result.running=running_; return result;
}
void NativeDriver::cancel() {
  std::lock_guard<std::mutex> lock(mutex_);
  target_=Target{}; target_.sequence=++generation_; target_.cancel=true;
  status_.planner.idle=true; status_.reset_active=false;
}
std::uint64_t NativeDriver::submit(const PinkPose &pose,double duration) {
  if(!validPose(pose)||!std::isfinite(duration)||duration<0)
    throw std::invalid_argument("Expected finite base-frame SE(3) and nonnegative duration");
  std::lock_guard<std::mutex> lock(mutex_);
  if(!running_) throw std::runtime_error("Native servo is not running: "+status_.error);
  if(status_.reset_active) throw std::logic_error("Reset owns the planner until completion or cancellation");
  target_=Target{}; target_.pose=pose; target_.duration=duration; target_.arrival=now();
  target_.sequence=++generation_; status_.reset_active=duration>0;
  return generation_;
}
std::uint64_t NativeDriver::submitJoints(const PinkArmVector &q) {
  const auto lower=planner_.model().lowerLimits(),upper=planner_.model().upperLimits();
  for(int i=0;i<6;++i) if(!std::isfinite(q[i])||q[i]<lower[i]||q[i]>upper[i])
    throw std::invalid_argument("Joint target outside CS66 limits");
  std::lock_guard<std::mutex> lock(mutex_);
  if(!running_||status_.reset_active) throw std::runtime_error("Servo not ready for joint target");
  target_=Target{}; target_.q=q; target_.joint=true; target_.arrival=now();
  target_.sequence=++generation_; return generation_;
}
void NativeDriver::planLoop() noexcept {
  auto next=Clock::now(); std::uint64_t consumed=0;
  try {
    while(running_) {
      const double begin=now(); Target target;
      { std::lock_guard<std::mutex> lock(mutex_); target=target_; }
      if(target.sequence>consumed) {
        // After idle/stale/cancel, re-anchor to measured q rather than a stale command.
        if(target.cancel || planner_.status().idle) planner_.seed(rtsi_->getActualJointPositions());
        if(target.cancel) planner_.cancel();
        else if(target.joint) planner_.submitJoints(target.q,target.arrival,target.sequence);
        else planner_.submit(target.pose,target.arrival,target.sequence,target.duration);
        consumed=target.sequence;
      }
      auto q=planner_.tick(begin); auto ps=planner_.status(); auto pose=planner_.pose();
      bool rollback=false;
      PinkArmVector published_q{};
      {
        std::lock_guard<std::mutex> lock(mutex_);
        // Cancellation/retarget while IK was running must not publish obsolete work.
        if(target.sequence==target_.sequence) {
          status_.planner=ps; status_.commanded_q=q; status_.commanded_pose=pose;
          status_.reset_active=target.duration>0 && ps.moving;
          command_time_=now();
        } else {
          rollback=true; published_q=status_.commanded_q;
        }
        if(now()-begin>config_.servoj_time) ++status_.planner_overruns;
      }
      if(rollback) planner_.seed(published_q);
      waitTick(next,config_.servoj_time);
    }
  } catch(const std::exception &e) { fail(e.what()); }
  catch(...) { fail("Unknown native planner failure"); }
}
void NativeDriver::sendLoop() noexcept {
  auto next=Clock::now(); double last_stamp=rtsi_->getTimestamp(),fresh=now();
  PinkArmVector last_sent;
  { std::lock_guard<std::mutex> lock(mutex_); last_sent=status_.commanded_q; }
  const auto velocity=planner_.model().velocityLimits();
  const auto lower=planner_.model().lowerLimits(),upper=planner_.model().upperLimits();
  try {
    while(running_) {
      double stamp=rtsi_->getTimestamp();
      if(std::isfinite(stamp)&&stamp>last_stamp) { last_stamp=stamp; fresh=now(); }
      if(!rtsi_->isConnected()||now()-fresh>config_.feedback_timeout)
        throw std::runtime_error("RTSI feedback stale/disconnected");
      const auto measured=rtsi_->getActualJointPositions();
      for(int i=0;i<6;++i)
        if(!std::isfinite(measured[i]) || measured[i]<lower[i] || measured[i]>upper[i])
          throw std::runtime_error("Measured joints outside CS66 model bounds");
      NativeStatus s; double updated;
      { std::lock_guard<std::mutex> lock(mutex_); s=status_; updated=command_time_; }
      if(now()-updated>config_.planner_timeout)
        throw std::runtime_error("Native planner deadline exceeded");
      bool ok;
      if(s.planner.idle) {
        ok=driver_->writeIdle(config_.command_timeout_ms);
        last_sent=measured;
      } else {
        auto q=s.commanded_q;
        // Sender-side gate remains necessary if intermediate planner samples were skipped.
        for(int i=0;i<6;++i) {
          double step=velocity[i]*config_.planner.velocity_scale*config_.servoj_time;
          q[i]=last_sent[i]+std::clamp(q[i]-last_sent[i],-step,step);
        }
        ok=driver_->writeServoj(q,config_.command_timeout_ms,false);
        if(ok) last_sent=q;
      }
      if(!ok) throw std::runtime_error("Elite reverse socket write failed");
      waitTick(next,config_.servoj_time);
    }
  } catch(const std::exception &e) { fail(e.what()); }
  catch(...) { fail("Unknown native sender failure"); }
  try { driver_->writeIdle(config_.command_timeout_ms); } catch(...) {}
}
void NativeDriver::moveJ(const PinkArmVector &q,double duration,int timeout_ms) {
  std::lock_guard<std::mutex> lifecycle(lifecycle_mutex_);
  if(running_) throw std::logic_error("Stop native servo before MoveJ");
  if(!std::isfinite(duration)||duration<=0||timeout_ms<=0) throw std::invalid_argument("Invalid MoveJ timing");
  planner_.model().forwardKinematics(0,q); // validates limits before any write
  auto result=std::make_shared<std::atomic<int>>(-999);
  driver_->setTrajectoryResultCallback([result](ELITE::TrajectoryMotionResult r) { result->store(static_cast<int>(r)); });
  try {
    if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::START,1,timeout_ms) ||
       !driver_->writeTrajectoryPoint(q,static_cast<float>(duration),0,false))
      throw std::runtime_error("MoveJ submission failed");
    double deadline=now()+duration+5;
    while(result->load()==-999) {
      if(now()>deadline) throw std::runtime_error("MoveJ timed out");
      if(!driver_->writeTrajectoryControlAction(ELITE::TrajectoryControlAction::NOOP,0,timeout_ms))
        throw std::runtime_error("MoveJ keepalive failed");
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if(result->load()!=static_cast<int>(ELITE::TrajectoryMotionResult::SUCCESS))
      throw std::runtime_error("Controller rejected/aborted MoveJ");
  } catch(...) { driver_->writeIdle(timeout_ms); throw; }
  driver_->writeIdle(timeout_ms);
}
}
