#include <ros/ros.h>
//#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <andbot/WheelCmd.h>
//#include <nav_msgs/Odometry.h>
#include <iostream>
using namespace std;
//double wheelSeparation = 0.22; for small lambo
double wheelSeparation = 0.393;
//double wheelSeparation = 0.3996;
double wheelRadius = 0.078;
//double wheelRadius = 0.0637; for small lambo
//double wheelRadius = 0.0625;

double vl = 0.0;
double vr = 0.0;

ros::Publisher cmd_wheel_angularVel_pub;
ros::Subscriber cmd_vel_sub;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
  andbot::WheelCmd wheel;
  geometry_msgs::Twist twist = twist_aux;
  double vel_x = twist_aux.linear.x;
  double vel_th = twist_aux.angular.z;
  double right_vel = 0.0;
  double left_vel = 0.0;

  // (vel_x, vel_th) --> (vl, vr)
  left_vel = (2*vel_x - vel_th * wheelSeparation) / 2 / wheelRadius;
  right_vel =(2*vel_x + vel_th * wheelSeparation) / 2 / wheelRadius;

  // publish to /cmd_wheel_angularVel
  wheel.speed1 = left_vel;
  wheel.speed2 = right_vel;
  wheel.mode1 = true;
  wheel.mode2 = true;
  cmd_wheel_angularVel_pub.publish(wheel);
  
}

int main(int argc, char** argv){
  ros::init(argc, argv, "mybot_base_controller");
  ros::NodeHandle n1, n2;
  cmd_vel_sub = n1.subscribe("/andbot/cmd_vel", 10, cmd_velCallback);
  cmd_wheel_angularVel_pub = n2.advertise<andbot::WheelCmd>("cmd_wheel_angularVel", 50);
  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}


