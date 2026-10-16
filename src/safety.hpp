#pragma once
#include "pink_ik.hpp"
#include <cmath>
#include <stdexcept>

namespace libpyelite {
// Pure validation used by the hardware loop and hardware-free fault injection.
inline void validateRobotFeedback(int robot_mode, int safety_mode,
    const PinkArmVector &wrench, const PinkArmVector &limits, bool check_mode=true) {
  // Elite SDK RobotMode::RUNNING=7, SafetyMode::{NORMAL=1,REDUCED=2}.
  if(check_mode && (robot_mode!=7 || (safety_mode!=1 && safety_mode!=2)))
    throw std::runtime_error("Robot fault/stop or non-running mode; manual recovery required");
  for(int i=0;i<6;++i) {
    if(!std::isfinite(limits[i])||limits[i]<0) throw std::invalid_argument("Invalid wrench threshold");
    if(limits[i]>0 && (!std::isfinite(wrench[i]) || std::abs(wrench[i])>limits[i]))
      throw std::runtime_error("Measured TCP wrench exceeded configured trip threshold or is invalid");
  }
}
}
