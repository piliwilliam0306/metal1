#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <math.h>
#include <angelbot/WheelCmd.h>
#include "twist2motor.h"


TwistToMotors::TwistToMotors()
{
	init_variables();
	get_parameters();

	ROS_INFO("Started Twist to Motor node");

	cmd_vel_sub = n.subscribe("/angelbot/cmd_vel",10, &TwistToMotors::twistCallback, this);
	cmd_wheel_angularVel_pub = n.advertise<angelbot::WheelCmd>("cmd_wheel_angularVel", 50);
}

void TwistToMotors::init_variables()
{
	leftwheel_angularVel = 0.0;
	rightwheel_angularVel = 0.0;
	vel_x = 0.0;
	vel_th= 0.0;
	wheelRadius = 0.075;
	wheelSeparation = 0.247;

	rate = 50;
	timeout_ticks = 2;
}


void TwistToMotors::get_parameters()
{
//
//	std::string xml_string;
//
//	// gets the location of the robot description on the parameter server
//	std::string full_param;
//	if (!n.searchParam(param, full_param))
//	{
//		ROS_ERROR("Could not find parameter %s on parameter server", param.c_str());
//		return false;
//	}
//	// read the robot description from the parameter server
//	if (!n.getParam(full_param, xml_string))
//	{
//		ROS_ERROR("Could not read parameter %s on parameter server", full_param.c_str());
//		return false;
//	}
//	return Model::initString(xml_string);

    if(n.getParam("rate", rate))
    {
		ROS_INFO_STREAM("Rate from param" << rate);
	}
    if(n.getParam("timeout_ticks", timeout_ticks))
    {
		ROS_INFO_STREAM("timeout_ticks from param" << timeout_ticks);
	}
	if(n.getParam("wheelSeparation", wheelSeparation))
	{
		ROS_INFO_STREAM("wheelSeparation from param" << wheelSeparation);
	}

	if(n.getParam("wheelRadius", wheelRadius))
	{
		ROS_INFO_STREAM("wheelRadius from param" << wheelRadius);
	}
}


void TwistToMotors::spin()
{
	ros::Rate repeat(rate);
	ros::Rate idle(10);

	ros::Time then = ros::Time::now();

	ticks_since_target = timeout_ticks;

	while (ros::ok())
	{
		while (ros::ok() && (ticks_since_target <= timeout_ticks))
		{
			spinOnce();
			repeat.sleep();
		}
		ros::spinOnce();
        idle.sleep();
	}
}

void TwistToMotors::spinOnce()
{
	angelbot::WheelCmd msg;
	// (vel_x, vel_th) --> (leftwheel_vel, rightwheel_vel)
	leftwheel_angularVel = (2*vel_x - vel_th * wheelSeparation) / 2 / wheelRadius;
	rightwheel_angularVel = (2*vel_x + vel_th * wheelSeparation) / 2 / wheelRadius;

//	ROS_INFO_STREAM("right = " << right << "\t" << "left = " << left << "dr"<< dr);

	// publish to /cmd_wheel_angularVel
	msg.speed1 = leftwheel_angularVel;
	msg.speed2 = rightwheel_angularVel;
	cmd_wheel_angularVel_pub.publish(msg);

	ticks_since_target += 1;
	ros::spinOnce();
}

void TwistToMotors::twistCallback(const geometry_msgs::Twist &twist_aux)
{
	ticks_since_target = 0;
	geometry_msgs::Twist twist = twist_aux;
	vel_x = twist_aux.linear.x;
	vel_th = twist_aux.angular.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"twist2motor");
	TwistToMotors obj;

	obj.spin();
}
