#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <andbot1dot2/WheelCmd.h>
#include <andbot1dot2/WheelFb.h>
#include <andbot1dot2/DriverState.h>
#include <geometry_msgs/Twist.h>

using namespace std;
double wheelRadius, wheelSeparation;

ros::Publisher cmd_wheel_volt_pub;
ros::Publisher feedback_Vel_pub;
ros::Subscriber cmd_vel_sub;
ros::Subscriber feedback_wheel_angularVel_sub;
double rate;

// variables for MIMO closed loop control
double vel_ref=0.0;
double vel_fb = 0.0;
double omega_ref = 0.0;
double omega_fb = 0.0;
double u_sum = 0.0;
double u_diff = 0.0;
double omega_fb_right = 0.0;
double omega_fb_left = 0.0;

// varibles for controllers
double vel_sum_error = 0.0;
double omega_sum_error = 0.0;

// saturation limit
const double Umax_volt = 12;
const double Umin_volt = -12;

//close loop: velocity loop controller
double vel_controller(double targetValue, double currentValue) 
{
  //static double last_error = 0;
  //long dT = 1/rate;
  float Kp;
  float Ki;
  double error;
  double pidTerm ;
  //double sum_error ;
  double constrained_pidTerm;
  
  error = targetValue - currentValue;
  
  // PI control
  Kp = 14.9624;
  Ki = 21.3962;
  vel_sum_error = vel_sum_error + error * (1/rate);

  if (vel_sum_error >= 10)        vel_sum_error = 10;
  else if (vel_sum_error<= -10)   vel_sum_error = -10;
  
  pidTerm = Kp * error + Ki * vel_sum_error;

  ROS_INFO("In vel_loop, we get error:[%], sum_error:[%] and pidTerm:[%]", error, sum_error,pidTerm);
  
  //saturation protection
  if (pidTerm >= 10)        constrained_pidTerm = 10;
  else if (pidTerm <= -10)  constrained_pidTerm = -10;
  else 	                               constrained_pidTerm = pidTerm ;
  
  return constrained_pidTerm;
}
double omega_controller(double targetValue, double currentValue) 
{
  //static double last_error = 0;
  //long dT = 1/rate;
  float Kp;
  float Ki;
  double error;
  double pidTerm;
  //double sum_error;
  double constrained_pidTerm;
  
  error = targetValue - currentValue;
  ROS_INFO_STREAM("omega_error=" << error);
  
  // PI control
  Kp = 4.4157;
  Ki = 5.8287;
  omega_sum_error = omega_sum_error + error * (1/rate);

  //pidTerm = Kp * error + Ki * omega_sum_error;

  if (omega_sum_error >= 10)  		omega_sum_error = 10;
  else if (omega_sum_error<= -10)  	omega_sum_error = -10;
  
  pidTerm = Kp * error + Ki * omega_sum_error;
  
  ROS_INFO("In omega_loop, we get error:[%], sum_error:[%] and pidTerm:[%]", error, sum_error,pidTerm);
  if (pidTerm >= 10)  		constrained_pidTerm = 10;
  else if (pidTerm <= -10) 	constrained_pidTerm = -10;
  else  	                        	constrained_pidTerm = pidTerm ;
  
  return constrained_pidTerm;
}

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
  andbot1dot2::WheelCmd wheel;
  geometry_msgs::Twist twist = twist_aux;
  double u_left = 0.0;
  double u_right = 0.0;
  double volt_friction_compensation = 0.7;
  double vel_FeedForward = 0.8;
  double omega_FeedForward = 4.9887/3.2393; //from model
  
  vel_ref = twist_aux.linear.x;
  omega_ref = twist_aux.angular.z;

  u_sum = vel_controller(0, 0);//vel_ref,vel_fb) + vel_ref * vel_FeedForward;
  u_diff = omega_controller(omega_ref, omega_fb) + omega_ref * omega_FeedForward ;
  
  if(u_sum >= Umax_volt) 	u_sum = Umax_volt;
  else if(u_sum <= Umin_volt)	u_sum = Umin_volt;
  else;

  if(u_diff >= Umax_volt)	u_diff = Umax_volt;
  else if(u_diff <= Umin_volt)	u_diff = Umin_volt;
  else;

  u_right = (u_sum + u_diff) / 2 ;
  u_left = (u_sum - u_diff) / 2 ; 
  
  // friction compensation
  if (vel_ref != 0.0 || omega_ref != 0.0)
  {
  	if (u_right < 0.0)	u_right = u_right - volt_friction_compensation;
  	else 				u_right = u_right + volt_friction_compensation;

  	if (u_left < 0.0) 	u_left = u_left - volt_friction_compensation;
  	else				u_left = u_left + volt_friction_compensation; 
  }
  else;				

  wheel.speed1 = u_left ;
  wheel.speed2 = u_right;

  cmd_wheel_volt_pub.publish(wheel);
}

void feedback_wheel_angularVelCallback(const andbot1dot2::WheelFb &wheel)
{
  geometry_msgs::Twist twist_aux;
  omega_fb_left = wheel.speed1;
  omega_fb_right  = wheel.speed2;
  
  //mobile robot kinematics transformation: differential drive
  vel_fb = wheelRadius / 2 * (omega_fb_right + omega_fb_left);
  omega_fb = wheelRadius / wheelSeparation * (omega_fb_right - omega_fb_left);

  twist_aux.linear.x = vel_fb;
  twist_aux.angular.z = omega_fb;
  feedback_Vel_pub.publish(twist_aux);
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
  feedback_wheel_angularVel_sub = n2.subscribe("feedback_wheel_angularVel", 10, feedback_wheel_angularVelCallback);
  feedback_Vel_pub = n2.advertise<geometry_msgs::Twist>("/andbot1dot2/feedback_Vel", 50);
  
  ros::Rate loop_rate(rate);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}


