#include "omni_robot_ros/cansend_transport.hpp"
#include <sys/wait.h>
#include <cerrno>
#include <chrono>
#include <iostream>
#include <stdexcept>
#define CHECK(x) do {if(!(x))throw std::runtime_error("Failed line "+std::to_string(__LINE__)+": " #x);} while(0)
int main(int argc,char** argv){try{
  CHECK(argc==2);std::string error;
  omni::CansendTransport transport(argv[1],"fake0",100);
  CHECK(transport.send(omni::encodeVelocity(omni::Velocity(-.1,.1,-.5),true,0),error));
  CHECK(!transport.send(omni::encodeVelocity(omni::Velocity(),false,1),error));
  const auto start=std::chrono::steady_clock::now();
  CHECK(!transport.send(omni::encodeVelocity(omni::Velocity(),false,2),error));
  CHECK(error=="cansend timed out");
  CHECK(std::chrono::steady_clock::now()-start<std::chrono::seconds(1));
  int status;CHECK(waitpid(-1,&status,WNOHANG)==-1&&errno==ECHILD);
  omni::CansendTransport missing("/no/such/cansend","fake0",10);
  CHECK(!missing.send(omni::encodeGetVersion(1),error));
  bool threw=false;try{omni::CansendTransport bad(argv[1],"can0;echo",10);}catch(const std::invalid_argument&){threw=true;}CHECK(threw);
  std::cout<<"PASS: argv/frame, success, nonzero exit, missing binary, timeout/child reaping, interface validation\n";
  return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
