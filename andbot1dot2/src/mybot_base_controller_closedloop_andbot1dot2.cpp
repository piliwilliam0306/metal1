#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <andbot1dot2/WheelCmd.h>
#include <andbot1dot2/DriverState.h>

using namespace std;
double wheelRadius, wheelSeparation;

ros::Publisher cmd_wheel_volt_pub;
ros::Subscriber cmd_vel_sub;
ros::Subscriber feedback_wheel_volt_sub;
double rate;

double vel_ref=0.0;
double vel_fb = 0.0;
double omega_ref = 0.0;
double omega_fb = 0.0;
double u_sum = 0.0;
double u_diff = 0.0;
double omega_fb_right = 0.0;
double omega_fb_left = 0.0;

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
  andbot1dot2::WheelCmd wheel;
  geometry_msgs::Twist twist = twist_aux;
  double vel_x = twist_aux.linear.x;
  double vel_th = twist_aux.angular.z;
  double right_vel = 0.0;
  double left_vel = 0.0;
  double u_left = 0.0;
  double u _right = 0.0;
  
  double Kp = 1.0;
  
  vel_ref = twist_aux.linear.x;
  omega_ref twist_aux.angular.z;
  u_sum = Kp * (vel_ref - vel_fb);
  u_diff = Kp * (omega_ref - omega_fb);
  
  u_right = (u_sum + u_diff) / 2 ;
  u_left = (u_sum - u_diff) / 2 ; 
  
  wheel.speed1 = u_left;
  wheel.speed2 = u_right;

  cmd_wheel_volt_pub.publish(wheel);
}
void feedback_wheel_voltCallback(const andbot1dot2::WheelFb &wheel)
{
	omega_fb_left = wheel.speed1;
  	omega_fb_right  = wheel.speed2;
  	
  	vel_fb = wheelRadius / 2 * (omega_fb_right + omega_fb_left);
  	omega_fb = wheelRadius / 2 / wheelSeparation * (omega_fb_right - omega_fb_left); 	 	
}

int main(int argc, char** argv){
  ros::init(argc, argv, "mybot_base_controller");
  ros::NodeHandle n1, n2;
  if(n1.getParam("rate", rate))
  {
	ROS_INFO_STREAM("rate from param =" << rate);
  }
  if(n1.getParam("wheelSeparation", wheelSeparation))
  {
	ROS_INFO_STREAM("wheelSeparation from param =" << wheelSeparation);
  }

  if(n1.getParam("wheelRadius", wheelRadius))
  {
	ROS_INFO_STREAM("wheelRadius from param =" << wheelRadius);
  }

  cmd_vel_sub = n1.subscribe("/andbot1dot2/cmd_vel", 10, cmd_velCallback);
  cmd_wheel_volt_pub = n2.advertise<andbot1dot2::WheelCmd>("cmd_wheel_volt", 50);
  feedback_wheel_volt_sub = n2.subscribe("feedback_wheel_volt", 10, feedback_wheel_voltCallback);
  
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}


