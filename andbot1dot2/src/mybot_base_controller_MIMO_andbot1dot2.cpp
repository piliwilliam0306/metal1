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

//variables for updatePid function

double Kalman = 0;

//close loop
double vel_controller(double targetValue, double currentValue) 
{
  static double last_error = 0;
  long dT = 0;
	float Kp;
	float Ki;
	double error;
	double pidTerm ;
	double sum_error ;
	double calculated_pidTerm;
	double constrained_pidterm;

//   if(WHEEL_SELECT==0) //left wheel      
//     targetValue = -targetValue;                             // 6.283 / 260 =0.0241653846153846
  
  error = targetValue - currentValue;
  ROS_INFO_STREAM("vel_error=" << error);
  
  Kp = 1040.0;//0.15;
  Ki = 0.0;
  sum_error = sum_error + error * dT;
  //sum_error = constrain(sum_error, -2000, 2000);
  if (sum_error >= 2000)
  {
  	sum_error = 2000;
  }
  else if (sum_error<= -2000)
  {
  	sum_error = 2000;
  }
  
  //  Serial.println(String("sum_error is =")+" "+String(sum_error));
  pidTerm = Kp * error + Ki * sum_error;

  calculated_pidTerm = pidTerm;//;pidTerm / 0.024165;                             // 6.283 / 260 =0.0241653846153846
	ROS_INFO_STREAM("vel_pidTerm=" << pidTerm);  
  //constrained_pidterm = constrain(calculated_pidTerm, -260, 260);
  if (calculated_pidTerm >= 520)
  {
  	constrained_pidterm = 520;
  }
  else if (calculated_pidTerm <= -520)
  {
  	constrained_pidterm = -520;
  }
  else
  {
  	constrained_pidterm = calculated_pidTerm ;
  }
  
  return constrained_pidterm;
}

double omega_controller(double targetValue, double currentValue) 
{
  static double last_error = 0;
    long dT = 0;
	float Kp;
	float Ki;
	double error;
	double pidTerm;
	double sum_error;
	double calculated_pidTerm;
	double constrained_pidterm;

//   if(WHEEL_SELECT==0) //left wheel      
//     targetValue = -targetValue;                             // 6.283 / 260 =0.0241653846153846
  
  error = targetValue - currentValue;
  ROS_INFO_STREAM("omega_error=" << error);
  Kp = 0.0;//0.15;
  Ki = 0.0;
  sum_error = sum_error + error * dT;
  //sum_error = constrain(sum_error, -2000, 2000);
  if (sum_error >= 2000)
  {
  	sum_error = 2000;
  }
  else if (sum_error<= -2000)
  {
  	sum_error = 2000;
  }
  
  //  Serial.println(String("sum_error is =")+" "+String(sum_error));
  pidTerm = Kp * error + Ki * sum_error;

  calculated_pidTerm = pidTerm;//;pidTerm / 0.024165;                             // 6.283 / 260 =0.0241653846153846
  
  ROS_INFO_STREAM("omega_pidTerm=" << pidTerm);
  //constrained_pidterm = constrain(calculated_pidTerm, -260, 260);
  if (calculated_pidTerm >= 520)
  {
  	constrained_pidterm = 520;
  }
  else if (calculated_pidTerm <= -520)
  {
  	constrained_pidterm = -520;
  }
  else
  {
  	constrained_pidterm = calculated_pidTerm ;
  }
  
  return constrained_pidterm;
}

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
  andbot1dot2::WheelCmd wheel;
  geometry_msgs::Twist twist = twist_aux;
  double u_left = 0.0;
  double u_right = 0.0;
  
  vel_ref = twist_aux.linear.x;
  omega_ref = twist_aux.angular.z;

  u_sum = vel_controller(vel_ref, vel_fb);//Kp * (vel_ref - vel_fb);
  u_diff = omega_controller(omega_ref, 0);// Kp * (omega_ref - omega_fb);
  
  u_right = (u_sum + u_diff) / 2 ;
  u_left = (u_sum - u_diff) / 2 ; 
  
  wheel.speed1 = u_left;
  wheel.speed2 = u_right;

  cmd_wheel_volt_pub.publish(wheel);
}
void feedback_wheel_angularVelCallback(const andbot1dot2::WheelFb &wheel)
{
	geometry_msgs::Twist twist_aux;
	omega_fb_left = wheel.speed1;
  	omega_fb_right  = wheel.speed2;
  	
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

