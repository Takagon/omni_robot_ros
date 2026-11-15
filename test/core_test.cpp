#include "omni_robot_ros/command_encoder.hpp"
#include "omni_robot_ros/command_gate.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
#define CHECK(x) do {if(!(x))throw std::runtime_error("Failed line " + std::to_string(__LINE__) + ": " #x);} while(0)
int main(){try{
  using namespace omni;
  CHECK(cansendArgument(encodeVelocity(Velocity(0,.1,0),true,1))=="200#0000640000000101");
  CHECK(cansendArgument(encodeVelocity(Velocity(0,0,.5),true,3))=="200#00000000F4010301");
  CHECK(cansendArgument(encodeVelocity(Velocity(-.1,.1,-.5),true,255))=="200#9CFF64000CFEFF01");
  CHECK(cansendArgument(encodeVelocity(Velocity(10,-10,10),true,0))=="200#F4010CFEE8030001");
  CHECK(cansendArgument(encodeVelocity(Velocity(.2,.3,.5),false,4))=="200#0000000000000400");
  CHECK(cansendArgument(encodeGetVersion(5))=="210#0105000000000000");
  const double nan=std::numeric_limits<double>::quiet_NaN();
  CHECK(encodeVelocity(Velocity(nan,0,0),true,1).data[7]==0);
  CHECK(encodeVelocity(Velocity(0,std::numeric_limits<double>::infinity(),0),true,1).data[7]==0);
  CHECK(encodeVelocity(Velocity(.0005,-.0005,0),true,1).data[0]==1);
  CHECK(encodeVelocity(Velocity(.0005,-.0005,0),true,1).data[2]==255);
  bool threw=false;try{validateLimits(Limits(.6,1));}catch(const std::invalid_argument&){threw=true;}CHECK(threw);
  threw=false;try{CommandGate bad(0,150);}catch(const std::invalid_argument&){threw=true;}CHECK(threw);
  std::uint8_t seq=255;seq++;CHECK(seq==0);

  CommandGate gate(150,150);Velocity out;
  CHECK(!gate.sample(0,out));
  gate.twist(Velocity(.1,0,0),1);CHECK(!gate.sample(1,out));
  gate.joy(true,2);gate.twist(Velocity(.1,0,0),2);CHECK(!gate.sample(2,out));
  gate.joy(false,3);gate.joy(true,4);CHECK(!gate.sample(4,out));
  gate.twist(Velocity(.1,0,0),5);CHECK(gate.sample(5,out)&&out.x==.1);
  CHECK(gate.sample(153,out));CHECK(!gate.sample(154,out)); // Joy timeout before Twist timeout
  gate.joy(true,155);gate.twist(Velocity(.1,0,0),155);CHECK(!gate.sample(155,out));
  gate.joy(false,156);gate.joy(true,157);gate.twist(Velocity(.2,0,0),158);CHECK(gate.sample(158,out));
  gate.joy(true,250);CHECK(gate.sample(307,out));CHECK(!gate.sample(308,out)); // cmd timeout
  gate.joy(false,309);gate.joy(true,310);gate.twist(Velocity(.2,0,0),311);
  gate.joy(false,312);CHECK(!gate.sample(312,out)&&out.x==0);
  gate.joy(true,313);CHECK(!gate.sample(313,out)); // no stale Twist on re-enable
  gate.twist(Velocity(.2,0,0),314);CHECK(gate.sample(314,out));
  gate.twist(Velocity(nan,0,0),315);CHECK(!gate.sample(315,out));
  gate.joy(false,316);gate.joy(true,317);gate.twist(Velocity(.2,0,0),318);
  gate.disarm();CHECK(!gate.sample(319,out));
  gate.joy(false,320);gate.joy(true,321);gate.twist(Velocity(.2,0,0),322);
  gate.twist(Velocity(.3,0,0),500);gate.joy(true,500);CHECK(!gate.sample(500,out)); // delayed callbacks cannot revive
  gate.joy(false,501);gate.joy(true,502);gate.twist(Velocity(.1,0,0),503);
  CHECK(!gate.sample(502,out)); // clock went backwards
  std::cout<<"PASS: packet bytes/units/sign/limits, version, startup, deadman, both watchdogs, rearm, NaN, sequence\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
