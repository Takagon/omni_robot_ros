#include "omni_robot_ros/command_gate.hpp"
#include "omni_robot_ros/cansend_transport.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <chrono>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace {
using Clock=std::chrono::steady_clock;
std::int64_t nowMs(){return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now().time_since_epoch()).count();}
struct Settings {
  std::string interface,executable;
  double rate=50,axis_limit=.5,yaw_limit=1;
  int cmd_timeout=150,joy_timeout=150,process_timeout=10;
  int enable=4,turbo=-1,axis_x=1,axis_y=0,axis_yaw=3;
  bool dry_run=false;
};
Settings load(ros::NodeHandle& p){
  Settings c;
  p.param<std::string>("can_interface",c.interface,"can0");
  p.param<std::string>("cansend_executable",c.executable,"cansend");
  p.param("send_rate_hz",c.rate,c.rate);
  p.param("max_axis_m_s",c.axis_limit,c.axis_limit);
  p.param("max_yaw_rad_s",c.yaw_limit,c.yaw_limit);
  p.param("cmd_timeout_ms",c.cmd_timeout,c.cmd_timeout);
  p.param("joy_timeout_ms",c.joy_timeout,c.joy_timeout);
  p.param("cansend_timeout_ms",c.process_timeout,c.process_timeout);
  p.param("enable_button",c.enable,c.enable);
  p.param("enable_turbo_button",c.turbo,c.turbo);
  p.param("axis_linear/x",c.axis_x,c.axis_x);
  p.param("axis_linear/y",c.axis_y,c.axis_y);
  p.param("axis_angular/yaw",c.axis_yaw,c.axis_yaw);
  p.param("dry_run",c.dry_run,c.dry_run);
  omni::validateLimits(omni::Limits(c.axis_limit,c.yaw_limit));
  if(!std::isfinite(c.rate)||c.rate<20||c.rate>100||
     c.cmd_timeout<=0||c.cmd_timeout>200||c.joy_timeout<=0||c.joy_timeout>200||
     c.process_timeout<=0||c.process_timeout>=1000.0/c.rate||
     c.enable<0||c.turbo < -1||c.axis_x<0||c.axis_y<0||c.axis_yaw<0)
    throw std::invalid_argument("Invalid rate/timeouts/joystick indices; see config YAML");
  return c;
}
class Bridge {
 public:
  Bridge(ros::NodeHandle& n,ros::NodeHandle& p,const Settings& c)
   :settings_(c),gate_(c.cmd_timeout,c.joy_timeout),transport_(c.executable,c.interface,c.process_timeout){
    joy_sub_=n.subscribe("joy",1,&Bridge::joy,this,ros::TransportHints().tcpNoDelay());
    twist_sub_=n.subscribe("cmd_vel",1,&Bridge::twist,this,ros::TransportHints().tcpNoDelay());
    frame_pub_=p.advertise<std_msgs::String>("tx_frame",1,false);
    ROS_INFO("CAN bridge: %s, %.1f Hz, dry_run=%s. Release enable, then hold it to drive.",
      c.interface.c_str(),c.rate,c.dry_run?"true":"false");
  }
  void tick(){
    omni::Velocity v;bool enabled;
    {std::lock_guard<std::mutex> guard(mutex_);enabled=gate_.sample(nowMs(),v);}
    if(!transmit(omni::encodeVelocity(v,enabled,sequence_++,omni::Limits(settings_.axis_limit,settings_.yaw_limit)))){
      std::lock_guard<std::mutex> guard(mutex_);gate_.disarm();
    }
  }
  void stop(){
    // Best effort on clean exit; STM32 watchdog handles crashes or link loss.
    for(int i=0;i<3;i++){
      transmit(omni::encodeVelocity(omni::Velocity(),false,sequence_++));
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }
 private:
  void joy(const sensor_msgs::Joy::ConstPtr& msg){
    bool valid=static_cast<std::size_t>(settings_.enable)<msg->buttons.size() &&
      (settings_.turbo<0||static_cast<std::size_t>(settings_.turbo)<msg->buttons.size());
    const int axes[]={settings_.axis_x,settings_.axis_y,settings_.axis_yaw};
    for(int a:axes) valid=valid && static_cast<std::size_t>(a)<msg->axes.size();
    if(valid) for(int a:axes) valid=valid&&std::isfinite(msg->axes[a]);
    std::lock_guard<std::mutex> guard(mutex_);
    if(!valid){gate_.disarm();ROS_WARN_THROTTLE(2,"Joy axes/buttons do not match joystick.yaml");return;}
    const bool enabled=msg->buttons[settings_.enable]!=0 ||
      (settings_.turbo>=0 && msg->buttons[settings_.turbo]!=0);
    gate_.joy(enabled,nowMs());
  }
  void twist(const geometry_msgs::Twist::ConstPtr& msg){
    std::lock_guard<std::mutex> guard(mutex_);
    if(!std::isfinite(msg->linear.z)||!std::isfinite(msg->angular.x)||!std::isfinite(msg->angular.y)||
       msg->linear.z!=0||msg->angular.x!=0||msg->angular.y!=0){gate_.disarm();return;}
    gate_.twist(omni::Velocity(msg->linear.x,msg->linear.y,msg->angular.z),nowMs());
  }
  bool transmit(const omni::Frame& f){
    std_msgs::String message;message.data=omni::cansendArgument(f);frame_pub_.publish(message);
    if(settings_.dry_run) return true;
    std::string error;
    if(!transport_.send(f,error)){ROS_ERROR_THROTTLE(1,"%s; disabled until enable is released/re-pressed",error.c_str());return false;}
    return true; // cansend success means queued locally, not acknowledged by the STM32 application.
  }
  Settings settings_;
  omni::CommandGate gate_;
  omni::CansendTransport transport_;
  std::mutex mutex_;
  std::uint8_t sequence_=0;
  ros::Subscriber joy_sub_,twist_sub_;
  ros::Publisher frame_pub_;
};
}
int main(int argc,char** argv){
  ros::init(argc,argv,"omni_can_bridge");
  try{
    ros::NodeHandle n,p("~");const Settings c=load(p);
    Bridge bridge(n,p,c);ros::AsyncSpinner spinner(1);spinner.start();
    const auto period=std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0/c.rate));
    auto next=Clock::now();
    while(ros::ok()){
      bridge.tick();next+=period;
      if(next<Clock::now()) {next=Clock::now()+period;ROS_WARN_THROTTLE(2,"CAN send deadline missed; no catch-up burst");}
      std::this_thread::sleep_until(next);
    }
    spinner.stop();bridge.stop();
    return 0;
  }catch(const std::exception& e){ROS_FATAL("%s",e.what());return 1;}
}
