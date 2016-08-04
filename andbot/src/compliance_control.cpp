#include <ros/ros.h>
#include <andbot/WheelCmd.h>
#include <andbot/WheelFb.h>
#include <iostream>
using namespace std;

ros::Publisher cmd_wheel_angularVel_pub;
ros::Subscriber feedback_wheel_angularVel_sub;

void cmd_velCallback(const andbot::WheelFb &wheel_fb)
{
  andbot::WheelCmd wheel_cmd;
  double right_vel = 0.0;
  double left_vel = 0.0;
  uint16_t right_current = 0;
  uint16_t left_current = 0;
  left_current = wheel_fb.current1;
  right_current = wheel_fb.current2;

  // (vel_x, vel_th) --> (vl, vr)
  if (left_current > 100)	left_vel = wheel_fb.speed1;
  else 				left_vel = 0.0;	
  if (right_current > 100)	right_vel = wheel_fb.speed2;
  else 				right_vel = 0.0; 	
  // publish to /cmd_wheel_angularVel
  wheel_cmd.speed1 = left_vel;
  wheel_cmd.speed2 = right_vel;
  wheel_cmd.driverstate = true;
  cmd_wheel_angularVel_pub.publish(wheel_cmd);
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "compliance_control");
  ros::NodeHandle n1, n2;
  feedback_wheel_angularVel_sub = n1.subscribe("feedback_wheel_angularVel", 10, cmd_velCallback);
  cmd_wheel_angularVel_pub = n2.advertise<andbot::WheelCmd>("cmd_wheel_angularVel", 50);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}


