#pragma once
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
};
struct NativeStatus {
  PlannerStatus planner;
  bool running{false};
  bool reset_active{false};
  std::string error;
  std::uint64_t planner_overruns{0};
  PinkArmVector commanded_q{};
  PinkPose commanded_pose{kIdentityPose};
};
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
  bool setPayload(double mass, const ELITE::vector3d_t &cog);
  bool getPrimaryPackage(std::shared_ptr<ELITE::PrimaryPackage> package, int timeout_ms);
 private:
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
  DriverConfig config_;
  Planner planner_;
  std::unique_ptr<ELITE::EliteDriver> driver_;
  std::shared_ptr<ELITE::RtsiIOInterface> rtsi_;
  std::atomic<bool> running_{false};
  std::thread planner_thread_, sender_thread_;
  mutable std::mutex mutex_;
  std::mutex lifecycle_mutex_;
  Target target_;
  NativeStatus status_;
  double command_time_{0};
  std::uint64_t generation_{0};
};
}
