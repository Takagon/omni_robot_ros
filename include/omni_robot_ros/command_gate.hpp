#ifndef OMNI_COMMAND_GATE_HPP
#define OMNI_COMMAND_GATE_HPP
#include "omni_robot_ros/command_encoder.hpp"
#include <cstdint>
namespace omni {
// Caller supplies steady-clock milliseconds and serializes access.
class CommandGate {
 public:
  CommandGate(std::int64_t command_timeout_ms, std::int64_t joy_timeout_ms);
  void joy(bool enable_pressed, std::int64_t now);
  void twist(const Velocity& value, std::int64_t now);
  void disarm(); // release and press enable again after faults/timeouts
  bool sample(std::int64_t now,Velocity& result);
 private:
  bool expired(std::int64_t now,std::int64_t stamp,std::int64_t timeout) const;
  std::int64_t cmd_timeout_,joy_timeout_,joy_time_=0,cmd_time_=0;
  bool have_joy_=false,pressed_=false,need_release_=true,have_cmd_=false;
  Velocity velocity_;
};
}
#endif
