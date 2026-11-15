#include "omni_robot_ros/command_gate.hpp"
#include <cmath>
#include <stdexcept>
namespace omni {
CommandGate::CommandGate(std::int64_t c,std::int64_t j):cmd_timeout_(c),joy_timeout_(j) {
  if(c<=0||j<=0) throw std::invalid_argument("Timeouts must be positive");
}
bool CommandGate::expired(std::int64_t now,std::int64_t t,std::int64_t duration) const {
  return now<t||now-t>=duration;
}
void CommandGate::disarm(){need_release_=true;have_cmd_=false;velocity_=Velocity();}
void CommandGate::joy(bool pressed,std::int64_t now){
  if(have_joy_&&expired(now,joy_time_,joy_timeout_)) disarm();
  have_joy_=true;joy_time_=now;
  if(!pressed){need_release_=false;have_cmd_=false;velocity_=Velocity();}
  // Require a new Twist after the enable transition (do not replay a latched Twist).
  if(pressed&&!pressed_) have_cmd_=false;
  pressed_=pressed;
}
void CommandGate::twist(const Velocity& v,std::int64_t now){
  if(!std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.yaw)){disarm();return;}
  if(have_cmd_&&expired(now,cmd_time_,cmd_timeout_)) disarm();
  if(!have_joy_||expired(now,joy_time_,joy_timeout_)){disarm();return;}
  if(need_release_||!pressed_) return;
  velocity_=v;cmd_time_=now;have_cmd_=true;
}
bool CommandGate::sample(std::int64_t now,Velocity& out){
  out=Velocity();
  if(!have_joy_||expired(now,joy_time_,joy_timeout_)){disarm();return false;}
  if(have_cmd_&&expired(now,cmd_time_,cmd_timeout_)){disarm();return false;}
  if(need_release_||!pressed_||!have_cmd_) return false;
  out=velocity_;return true;
}
}
