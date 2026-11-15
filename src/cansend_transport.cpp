#include "omni_robot_ros/cansend_transport.hpp"
#include <spawn.h>
#include <sys/wait.h>
#include <signal.h>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <utility>
extern char **environ;
namespace omni {
CansendTransport::CansendTransport(std::string exe,std::string dev,int timeout)
 :executable_(std::move(exe)),interface_(std::move(dev)),timeout_ms_(timeout){
  if(executable_.empty()||interface_.empty()||interface_.size()>15||timeout_ms_<=0)
    throw std::invalid_argument("Invalid cansend executable/interface/timeout");
  for(char c:interface_) if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||
    (c>='0'&&c<='9')||c=='_'||c=='-'||c=='.')) throw std::invalid_argument("Invalid CAN interface name");
}
bool CansendTransport::send(const Frame& f,std::string& error) const {
  const std::string argument=cansendArgument(f);
  char* argv[]={const_cast<char*>(executable_.c_str()),const_cast<char*>(interface_.c_str()),
    const_cast<char*>(argument.c_str()),nullptr};
  // No shell interpretation. One bounded child at a time; no detached sender.
  pid_t pid;
  int rc=posix_spawnp(&pid,executable_.c_str(),nullptr,nullptr,argv,environ);
  if(rc){error="cansend spawn: "+std::string(std::strerror(rc));return false;}
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms_);
  int status=0;
  for(;;){
    pid_t result=waitpid(pid,&status,WNOHANG);
    if(result==pid){
      if(WIFEXITED(status)&&WEXITSTATUS(status)==0){error.clear();return true;}
      error="cansend failed (wait status="+std::to_string(status)+")";return false;
    }
    if(result<0&&errno!=EINTR){
      const int saved=errno;
      if(saved!=ECHILD){kill(pid,SIGKILL);while(waitpid(pid,&status,0)<0&&errno==EINTR){}}
      error="cansend wait: "+std::string(std::strerror(saved));return false;
    }
    if(std::chrono::steady_clock::now()>=deadline){
      kill(pid,SIGKILL);while(waitpid(pid,&status,0)<0&&errno==EINTR){}
      error="cansend timed out";return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
}
