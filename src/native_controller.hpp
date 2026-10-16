#pragma once
#include "compliance.hpp"
#include "planner.hpp"
#include <Elite/EliteDriver.hpp>
#include <Elite/RtsiIOInterface.hpp>
#include <atomic>
#include <mutex>
#include <thread>

namespace libpyelite {
struct DriverConfig : ELITE::EliteDriverConfig {
  DriverConfig() { servoj_time=0.004; headless_mode=true; }
  std::string model_path;
  PinkPose tool{kIdentityPose}; // tool0 <- configured TCP, m, row-major SE(3)
  PinkIKOptions ik;
  PlannerConfig planner;
  int command_timeout_ms{500};
  // Reverse-socket writes fail transiently right after the external control
  // script comes up; fault only after this many consecutive failed periods.
  int max_consecutive_write_failures{1};
  double feedback_timeout{0.1};
  double planner_timeout{0.1};
  double fk_position_tolerance{0.005};
  double fk_orientation_tolerance{0.05};
  bool check_robot_safety{true};
  PinkArmVector max_contact_wrench{}; // zero disables that axis; N, Nm. RTSI actual_TCP_force
  // does not respond to external force on the CS66; prefer contact_monitor.
  // Joint-current residual collision detection (any link). Only its collision
  // trip is used here: the admittance offset it computes is ignored.
  ComplianceConfig contact_monitor;
  double max_start_joint_speed{0.02}; // require stationary seed

};
struct NativeStatus {
  PlannerStatus planner;
  bool running{false};
  bool reset_active{false};
  std::string error;
  std::uint64_t planner_overruns{0};
  PinkArmVector commanded_q{}, sent_q{}, sent_velocity{}, sent_acceleration{};
  PinkPose commanded_pose{kIdentityPose};
  ComplianceStatus contact;  // contact_monitor estimate; biased=false while zeroing
  double max_sender_gap_s{0}, max_write_s{0};  // longest sender period and reverse-socket write seen
};
class StationCoordinator;
class NativeDriver {
 public:
  explicit NativeDriver(DriverConfig config);
  ~NativeDriver();
  bool isRobotConnected();
  bool sendExternalControlScript();
  void startServo(std::shared_ptr<ELITE::RtsiIOInterface> rtsi);
  void stopServo();
  void stopControl(int timeout_ms);
  void cancel();
  std::uint64_t submit(const PinkPose &pose, double duration=0);
  std::uint64_t submitJoints(const PinkArmVector &q);
  NativeStatus status() const;
  void moveJ(const PinkArmVector &q, double duration, int timeout_ms);
  bool writeIdle(int timeout_ms);
  // Coordinated mode: no planner thread of its own; a StationCoordinator plans
  // both arms each period under the station collision layer. Set before startServo.
  void setExternalPlanning(bool external);
  bool externalPlanning() const { return external_planning_; }
  // Re-zero the contact monitor; the arm must stay still and untouched for bias_time.
  void rebiasContact();
  bool setPayload(double mass, const ELITE::vector3d_t &cog);
  bool getPrimaryPackage(std::shared_ptr<ELITE::PrimaryPackage> package, int timeout_ms);
 private:
  friend class StationCoordinator;
  struct Target {
    PinkPose pose{kIdentityPose};
    PinkArmVector q{};
    double arrival{0}, duration{0};
    std::uint64_t sequence{0};
    bool joint{false}, cancel{false};
  };
  void planLoop() noexcept;
  void sendLoop() noexcept;
  void fail(const std::string &message) noexcept;
  void checkFeedbackSafety();
  DriverConfig config_;
  Planner planner_;
  std::unique_ptr<ELITE::EliteDriver> driver_;
  std::shared_ptr<ELITE::RtsiIOInterface> rtsi_;
  std::unique_ptr<ComplianceController> contact_;
  std::atomic<bool> running_{false}, rebias_contact_{false}, external_planning_{false};
  // Coordinated planning, called by StationCoordinator with lifecycle_mutex_ held.
  void consumeTarget(std::uint64_t &consumed, Target &target);
  void publish(const Target &target, double begin);
  std::thread planner_thread_, sender_thread_;
  mutable std::mutex mutex_;
  std::mutex lifecycle_mutex_;
  Target target_;
  NativeStatus status_;
  double command_time_{0};
  std::uint64_t generation_{0};
};
}
