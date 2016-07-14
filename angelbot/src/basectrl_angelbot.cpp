#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include "basectrl_angelbot.h"

using namespace std;

basectrl_angelbot::basectrl_angelbot()
{
	init_variables();
	get_parameters();

	cmd_vel_sub = n1.subscribe("/angelbot/cmd_vel", 10, cmd_velCallback, this);
	cmd_wheel_angularVel_pub = n2.advertise<geometry_msgs::Vector3>("cmd_wheel_angularVel", 50);
}

void basectrl_angelbot::get_parameters()
{
	if(n1.getParam("rate", rate))
	{
		ROS_INFO_STREAM("Rate from param" << rate);
	}

    if(n1.getParam("wheelSeparation", wheelSeparation))
    {
		ROS_INFO_STREAM("wheelSeparation from param" << wheelSeparation);
	}

    if(n1.getParam("wheelRadius", wheelRadius))
    {
		ROS_INFO_STREAM("wheelRadius from param" << wheelRadius);
	}
}

void basectrl_angelbot::init_variables()
{
	leftwheel_angularVel = 0.0;
	rightwheel_angularVel = 0.0;
	vel_x = 0.0;
	vel_th= 0.0;
}

void basectrl_angelbot::cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
  geometry_msgs::Twist twist = twist_aux;
  vel_x = twist_aux.linear.x;
  vel_th = twist_aux.angular.z;
}

void basectrl_angelbot::spinOnce()
{
	geometry_msgs::Vector3 vector3;

	// (vel_x, vel_th) --> (leftwheel_vel, rightwheel_vel)
	leftwheel_angularVel = (2*vel_x - vel_th * wheelSeparation) / 2 / wheelRadius;
	rightwheel_angularVel = (2*vel_x + vel_th * wheelSeparation) / 2 / wheelRadius;

	// publish to /cmd_wheel_angularVel
	vector3.x = leftwheel_angularVel;
	vector3.y = rightwheel_angularVel;
	cmd_wheel_angularVel_pub.publish(vector3);

	ros::spinOnce();
}
void basectrl_angelbot::spin()
{
	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}
int main(int argc, char** argv){
  ros::init(argc, argv, "basectrl_angelbot");

  basectrl_angelbot obj;

  obj.spin();
}


