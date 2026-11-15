#ifndef OMNI_COMMAND_ENCODER_HPP
#define OMNI_COMMAND_ENCODER_HPP
#include <array>
#include <cstdint>
#include <string>
namespace omni {
struct Velocity {
  double x, y, yaw; // body frame: m/s, m/s, rad/s
  Velocity(double vx=0, double vy=0, double w=0):x(vx),y(vy),yaw(w){}
};
struct Limits {
  double axis_m_s, yaw_rad_s;
  Limits(double axis=0.5,double yaw=1.0):axis_m_s(axis),yaw_rad_s(yaw){}
};
struct Frame { std::uint16_t id; std::array<std::uint8_t,8> data; };
void validateLimits(const Limits& limits);
Frame encodeVelocity(const Velocity& velocity, bool enabled, std::uint8_t sequence,
                     const Limits& limits=Limits());
Frame encodeGetVersion(std::uint8_t request_id);
std::string cansendArgument(const Frame& frame);
}
#endif
