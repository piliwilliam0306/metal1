#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
using namespace std;
double wheelSeparation = 0.393;
double wheelRadius = 0.078;
double omega_right = 0.0;
double omega_left = 0.0;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double vel_real = 0.0;
double omega_real = 0.0;

ros::Publisher cmd_wheel_angularPWM_pub;
ros::Publisher real_vel_pub;
ros::Subscriber cmd_vel_sub;
ros::Subscriber feedback_wheel_angularVel_sub;
ros::Time current_time, last_time;

void feedback_wheel_angularVelCallback(const geometry_msgs::Vector3 &vector3)
{
  current_time = ros::Time::now();
  double dt = (current_time - last_time).toSec();
  last_time = current_time;
  //ROS_INFO("dt:%f",dt*1000);//ms 

  geometry_msgs::Twist twist_real;
  omega_left = vector3.x;
  omega_right  = vector3.y;

  current_time = ros::Time::now();
  vel_real = wheelRadius * (omega_right + omega_left) / 2;
  omega_real = wheelRadius * (omega_right - omega_left) / wheelSeparation;

  twist_real.linear.x = vel_real;
  twist_real.angular.z = omega_real;
  real_vel_pub.publish(twist_real);

  double delta_x = vel_real * cos(th) * dt;
  double delta_y = vel_real * sin(th) * dt;
  double delta_th = omega_real * dt;
  x += delta_x;
  y += delta_y;
  th += delta_th;
}



void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
  geometry_msgs::Vector3 vector3;
  geometry_msgs::Twist twist = twist_aux;
  double vel_x = twist_aux.linear.x;
  double vel_th = twist_aux.angular.z;
  double right_vel = 0.0;
  double left_vel = 0.0;
  double left_pwm = 0.0;
  double right_pwm = 0.0;

  left_vel = (2*vel_x - vel_th * wheelSeparation) / 2 / wheelRadius;
  right_vel =(2*vel_x + vel_th * wheelSeparation) / 2 / wheelRadius;
  
  //left_pwm = left_vel*float(255/8.9);
  //right_pwm = right_vel*float(255/8.9);
  double us = (vel_x - vel_real)*30;
  double ud = (vel_th - omega_real)*5;
  left_pwm = 0.5*(us-ud)*double(255/12);
  if (left_pwm>=255)
	left_pwm=255;
  else if (left_pwm<=-255)
	left_pwm=-255;

  right_pwm = 0.5*(us+ud)*double(255/12);
  if (right_pwm>=255)
	right_pwm=255;
  else if (right_pwm<=-255)
	right_pwm=-255;
  
  
  // publish to /cmd_wheel_angularVel
  vector3.x = left_pwm;
  vector3.y = right_pwm;
  cmd_wheel_angularPWM_pub.publish(vector3);
  ROS_INFO("V:%f W:%f  V_real:%f  W_real:%f  pwmL:%f  pwmR:%f us:%f ud:%f",vel_x,vel_th,vel_real,omega_real,left_pwm,right_pwm,us,ud); 
  //ROS_INFO("V:%f W:%f VR:%f VW:%f",vel_x,vel_th,vel_real,omega_real); 

}

int main(int argc, char** argv){
  ros::init(argc, argv, "mimo_base_controller");
  ros::NodeHandle n1, n2;
  cmd_vel_sub = n1.subscribe("/andbot/cmd_vel", 10, cmd_velCallback);
  real_vel_pub = n1.advertise<geometry_msgs::Twist>("/andbot/real_vel", 1);
  cmd_wheel_angularPWM_pub = n2.advertise<geometry_msgs::Vector3>("/cmd_wheel_pwm", 1);

  feedback_wheel_angularVel_sub = n1.subscribe("/feedback_wheel_angularVel", 1, feedback_wheel_angularVelCallback);



  ros::Rate loop_rate(100);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}


