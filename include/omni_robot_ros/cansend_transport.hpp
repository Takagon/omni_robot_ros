#ifndef OMNI_CANSEND_TRANSPORT_HPP
#define OMNI_CANSEND_TRANSPORT_HPP
#include "omni_robot_ros/command_encoder.hpp"
#include <string>
namespace omni {
class CansendTransport {
 public:
  CansendTransport(std::string executable,std::string interface,int timeout_ms);
  bool send(const Frame& frame,std::string& error) const;
 private:
  std::string executable_,interface_;
  int timeout_ms_;
};
}
#endif
