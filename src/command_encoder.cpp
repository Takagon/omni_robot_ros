#include "omni_robot_ros/command_encoder.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
namespace omni {
void validateLimits(const Limits& l) {
  if(!std::isfinite(l.axis_m_s)||!std::isfinite(l.yaw_rad_s)||
     l.axis_m_s<=0||l.axis_m_s>0.5||l.yaw_rad_s<=0||l.yaw_rad_s>1.0)
    throw std::invalid_argument("Limits must fit STM32: 0 < axis <= 0.5 m/s, 0 < yaw <= 1 rad/s");
}
static void putSigned(std::array<std::uint8_t,8>& b,unsigned i,double value) {
  const std::uint16_t u=static_cast<std::uint16_t>(static_cast<std::int16_t>(std::lround(value)));
  b[i]=static_cast<std::uint8_t>(u&0xff); b[i+1]=static_cast<std::uint8_t>(u>>8);
}
Frame encodeVelocity(const Velocity& v,bool enable,std::uint8_t seq,const Limits& l) {
  validateLimits(l);
  Frame f={0x200,{{0,0,0,0,0,0,seq,0}}};
  if(!enable||!std::isfinite(v.x)||!std::isfinite(v.y)||!std::isfinite(v.yaw)) return f;
  putSigned(f.data,0,std::max(-l.axis_m_s,std::min(l.axis_m_s,v.x))*1000.0);
  putSigned(f.data,2,std::max(-l.axis_m_s,std::min(l.axis_m_s,v.y))*1000.0);
  putSigned(f.data,4,std::max(-l.yaw_rad_s,std::min(l.yaw_rad_s,v.yaw))*1000.0);
  f.data[7]=1; return f;
}
Frame encodeGetVersion(std::uint8_t id) {return Frame{0x210,{{1,id,0,0,0,0,0,0}}};}
std::string cansendArgument(const Frame& f) {
  if(f.id>0x7ff) throw std::invalid_argument("Only standard CAN IDs supported");
  std::ostringstream s; s<<std::uppercase<<std::hex<<std::setfill('0')<<std::setw(3)<<f.id<<'#';
  for(auto b:f.data) s<<std::setw(2)<<static_cast<unsigned>(b);
  return s.str();
}
}  // namespace omni
