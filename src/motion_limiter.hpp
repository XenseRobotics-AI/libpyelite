#pragma once
#include "pink_ik.hpp"
#include <ruckig/ruckig.hpp>
#include <stdexcept>
#include <cmath>
#include <sstream>

namespace libpyelite {
// Stateful command trajectory. Limits describe commands, not measured servo dynamics.
class MotionLimiter {
 public:
  MotionLimiter(double dt, PinkArmVector velocity, PinkArmVector acceleration,
                PinkArmVector jerk, PinkArmVector lower, PinkArmVector upper)
      : otg_(dt), dt_(dt), lower_(lower), upper_(upper), velocity_(velocity), acceleration_(acceleration), jerk_(jerk) {
    if(!std::isfinite(dt)||dt<=0) throw std::invalid_argument("Invalid trajectory period");
    for(int i=0;i<6;++i)
      if(!std::isfinite(lower[i])||!std::isfinite(upper[i])||lower[i]>=upper[i])
        throw std::invalid_argument("Invalid joint position bounds");
    for(int i=0;i<6;++i)
      for(double x:{velocity[i],acceleration[i],jerk[i]})
        if(!std::isfinite(x)||x<=0) throw std::invalid_argument("Motion limits must be six positive finite values");
    input_.max_velocity=velocity; input_.max_acceleration=acceleration; input_.max_jerk=jerk;
  }
  void seed(const PinkArmVector &q) {
    for(int i=0;i<6;++i)
      if(!std::isfinite(q[i])||q[i]<lower_[i]||q[i]>upper_[i]) throw std::invalid_argument("Invalid trajectory seed");
    finished_=true;
    input_.current_position=q; input_.current_velocity.fill(0); input_.current_acceleration.fill(0);
    otg_.reset();
    sent_=q; sent_velocity_.fill(0); sent_acceleration_.fill(0);
  }
  void reanchor(const PinkArmVector &q,const PinkArmVector &v,const PinkArmVector &a) {
    seed(q);
    for(int i=0;i<6;++i) if(!std::isfinite(v[i])||!std::isfinite(a[i])) throw std::invalid_argument("Invalid reanchor derivatives");
    input_.current_velocity=v; input_.current_acceleration=a;
    sent_velocity_=v; sent_acceleration_=a;
  }
  // Sender gate. A sample from an upstream limiter with the same period and
  // limits is already a feasible continuation; pass it through unchanged so the
  // limits are not applied twice in series (each pass costs tracking lag). Only
  // a sample whose finite differences from what was last sent break the limits
  // (a missed or repeated planner tick, a reseeded planner) is tracked instead.
  // hard_only: check only position and velocity. Used behind the station
  // collision gate, whose acceleration bound is deliberately soft (it may brake
  // harder near an obstacle); smoothing that braking here would undo it.
  PinkArmVector gate(const PinkArmVector &target,const PinkArmVector &velocity,const PinkArmVector &acceleration,
                     bool hard_only=false) {
    constexpr double kSlack=1.05;
    bool feasible=true;
    PinkArmVector v{},a{};
    for(int i=0;i<6;++i) {
      if(!std::isfinite(target[i])) throw std::invalid_argument("Nonfinite trajectory target");
      v[i]=(target[i]-sent_[i])/dt_; a[i]=(v[i]-sent_velocity_[i])/dt_;
      const double j=(a[i]-sent_acceleration_[i])/dt_;
      feasible=feasible && target[i]>=lower_[i] && target[i]<=upper_[i] &&
        std::abs(v[i])<=velocity_[i] &&
        (hard_only || (std::abs(a[i])<=acceleration_[i]*kSlack && std::abs(j)<=jerk_[i]*kSlack));
    }
    passed_=feasible;
    if(!feasible) {
      PinkArmVector tv=velocity,ta=acceleration;
      for(int i=0;i<6;++i) {
        tv[i]=std::clamp(tv[i],-velocity_[i],velocity_[i]); ta[i]=std::clamp(ta[i],-acceleration_[i],acceleration_[i]);
      }
      const auto out=track(target,tv,ta);
      for(int i=0;i<6;++i) { v[i]=(out[i]-sent_[i])/dt_; a[i]=(v[i]-sent_velocity_[i])/dt_; }
      sent_=out; sent_velocity_=v; sent_acceleration_=a;
      return out;
    }
    input_.current_position=target; input_.current_velocity=velocity; input_.current_acceleration=acceleration;
    finished_=true;
    sent_=target; sent_velocity_=v; sent_acceleration_=a;
    return target;
  }
  bool passed() const { return passed_; }
  PinkArmVector step(const PinkArmVector &target, bool stop=false) {
    for(double x:target) if(!std::isfinite(x)) throw std::invalid_argument("Nonfinite trajectory target");
    input_.control_interface=stop?ruckig::ControlInterface::Velocity:ruckig::ControlInterface::Position;
    input_.target_position=target; input_.target_velocity.fill(0); input_.target_acceleration.fill(0);
    return update();
  }
  PinkArmVector track(const PinkArmVector &target,const PinkArmVector &velocity,const PinkArmVector &acceleration) {
    for(int i=0;i<6;++i) if(!std::isfinite(target[i])||!std::isfinite(velocity[i])||!std::isfinite(acceleration[i]))
      throw std::invalid_argument("Nonfinite trajectory target state");
    input_.control_interface=ruckig::ControlInterface::Position;
    input_.target_position=target; input_.target_velocity=velocity; input_.target_acceleration=acceleration;
    return update();
  }
  // Follow a moving joint reference without stopping at each sample. Ruckig's
  // position interface treats every target as a fixed point, so a moving
  // reference is always reached late; its tracking interface is Pro-only. Here the
  // velocity interface takes feed-forward reference velocity plus a correction
  // that never asks for more than the braking distance allows, and Ruckig keeps
  // the acceleration and jerk limits.
  PinkArmVector follow(const PinkArmVector &target,const PinkArmVector &velocity,double gain) {
    if(!std::isfinite(gain)||gain<=0) throw std::invalid_argument("Invalid tracking gain");
    PinkArmVector command{};
    for(int i=0;i<6;++i) {
      if(!std::isfinite(target[i])||!std::isfinite(velocity[i])) throw std::invalid_argument("Nonfinite trajectory target state");
      const double error=target[i]-input_.current_position[i];
      const double correction=std::min(gain*std::abs(error),std::sqrt(acceleration_[i]*std::abs(error)));
      // Never command a speed that could not brake before a position limit
      // (the jerk ramp is covered by using half of the acceleration).
      const auto room=[&](double distance) {
        return std::sqrt(acceleration_[i]*std::max(0.0,distance-kLimitMargin));
      };
      command[i]=std::clamp(velocity[i]+std::copysign(correction,error),
        -std::min(velocity_[i],room(input_.current_position[i]-lower_[i])),
        std::min(velocity_[i],room(upper_[i]-input_.current_position[i])));
    }
    input_.control_interface=ruckig::ControlInterface::Velocity;
    input_.target_velocity=command; input_.target_acceleration.fill(0);
    return update(false);
  }
  PinkArmVector update(bool check_envelope=true) {
    // Round-off residue (re-anchoring adds correction/dt, about 1e-15 rad/s) on
    // a state already at its target makes Ruckig fail its time calculation
    // (-111) rather than report Finished. Flush what no servo can resolve.
    for(int i=0;i<6;++i) {
      // A state handed over from outside (gate pass-through, re-anchoring) may sit
      // marginally past a limit; Ruckig rejects that input (-100) instead of braking.
      input_.current_velocity[i]=std::clamp(input_.current_velocity[i],-velocity_[i],velocity_[i]);
      input_.current_acceleration[i]=std::clamp(input_.current_acceleration[i],-acceleration_[i],acceleration_[i]);
      // At the velocity limit, acceleration still pushing outward cannot be braked within it.
      if(std::abs(input_.current_velocity[i])>=velocity_[i]*(1-1e-9) && input_.current_velocity[i]*input_.current_acceleration[i]>0)
        input_.current_acceleration[i]=0;
      if(std::abs(input_.current_velocity[i])<kTinyVelocity) input_.current_velocity[i]=0;
      if(std::abs(input_.current_acceleration[i])<kTinyAcceleration) input_.current_acceleration[i]=0;
      if(std::abs(input_.target_velocity[i])<kTinyVelocity) input_.target_velocity[i]=0;
      if(input_.control_interface==ruckig::ControlInterface::Position &&
         std::abs(input_.target_position[i]-input_.current_position[i])<kTinyPosition)
        input_.target_position[i]=input_.current_position[i];
    }
    // Already at a still target (round-off apart): Ruckig cannot time a
    // zero-length move (-111), so settle on it directly.
    if(input_.control_interface==ruckig::ControlInterface::Position) {
      bool settled=true;
      for(int i=0;i<6;++i)
        settled=settled && std::abs(input_.target_position[i]-input_.current_position[i])<kSettlePosition &&
          std::abs(input_.current_velocity[i])<kSettleVelocity && std::abs(input_.current_acceleration[i])<kSettleAcceleration &&
          input_.target_velocity[i]==0 && input_.target_acceleration[i]==0;
      if(settled) {
        for(int i=0;i<6;++i) {
          const double v=(input_.target_position[i]-sent_[i])/dt_;
          sent_acceleration_[i]=(v-sent_velocity_[i])/dt_; sent_velocity_[i]=v;
        }
        input_.current_position=input_.target_position;
        input_.current_velocity.fill(0); input_.current_acceleration.fill(0);
        sent_=input_.current_position; finished_=true;
        return input_.current_position;
      }
    }
    auto result=otg_.update(input_,output_);
    if(static_cast<int>(result)<0) {
      std::ostringstream message;
      message<<"Jerk-limited trajectory infeasible (Ruckig "<<static_cast<int>(result)<<", "
             <<(input_.control_interface==ruckig::ControlInterface::Velocity?"velocity":"position")<<" interface)";
      for(int i=0;i<6;++i)
        message<<"; J"<<i+1<<" p="<<input_.current_position[i]<<" v="<<input_.current_velocity[i]
               <<" a="<<input_.current_acceleration[i]<<" -> p="<<input_.target_position[i]
               <<" v="<<input_.target_velocity[i];
      throw std::runtime_error(message.str());
    }
    // Ruckig community edition does not enforce position limits. Validate the
    // entire generated trajectory, including braking overshoot, before publishing.
    // A velocity-following trajectory is not a braking envelope; follow() bounds
    // its speed by the braking distance instead, and the published sample is checked.
    if(check_envelope) {
      auto extrema=output_.trajectory.get_position_extrema();
      for(int i=0;i<6;++i)
        if(extrema[i].min<lower_[i]-1e-10 || extrema[i].max>upper_[i]+1e-10)
          throw std::runtime_error("Trajectory braking envelope exceeds joint limits");
    }
    for(int i=0;i<6;++i)
      if(output_.new_position[i]<lower_[i] || output_.new_position[i]>upper_[i])
        throw std::runtime_error("Trajectory sample exceeds joint limits");
    output_.pass_to_input(input_);
    finished_=result==ruckig::Result::Finished;
    for(int i=0;i<6;++i) {
      const double v=(input_.current_position[i]-sent_[i])/dt_;
      sent_acceleration_[i]=(v-sent_velocity_[i])/dt_; sent_velocity_[i]=v;
    }
    sent_=input_.current_position;
    return input_.current_position;
  }
  PinkArmVector velocity() const { return input_.current_velocity; }
  PinkArmVector acceleration() const { return input_.current_acceleration; }
  bool finished() const { return finished_; }
 private:
  static constexpr double kLimitMargin=1e-3;  // rad kept clear of a position limit while following
  static constexpr double kTinyPosition=1e-12, kTinyVelocity=1e-12, kTinyAcceleration=1e-9;  // rad, /s, /s^2
  static constexpr double kSettlePosition=1e-9, kSettleVelocity=1e-6, kSettleAcceleration=1e-4;  // rad, /s, /s^2
  ruckig::Ruckig<6> otg_;
  ruckig::InputParameter<6> input_;
  ruckig::OutputParameter<6> output_;
  double dt_;
  PinkArmVector lower_,upper_,velocity_,acceleration_,jerk_;
  // Finite-difference state of the published samples, for gate().
  PinkArmVector sent_{},sent_velocity_{},sent_acceleration_{};
  bool finished_{true}, passed_{false};
};
}
