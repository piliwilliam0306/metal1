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
	get_node_params();

	ROS_INFO("Started Twist to Motor node");

	cmd_vel_sub = n.subscribe("/angelbot/cmd_vel",10, &TwistToMotors::twistCallback, this);
	cmd_wheel_angularVel_pub = n.advertise<angelbot::WheelCmd>("cmd_wheel_angularVel", 50);
}

void TwistToMotors::init_variables()
{
	left_omega = 0.0;
	right_omega = 0.0;
	vel = 0.0;
	omega= 0.0;

	rate = 50;
	timeout_ticks = 2;
}


void TwistToMotors::get_node_params()
{
    if(n.getParam("rate", rate))
    {
		ROS_INFO_STREAM("Rate from param" << rate);
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

	while (ros::ok())
	{
		spinOnce();
        idle.sleep();
	}
}

void TwistToMotors::spinOnce()
{
	angelbot::WheelCmd msg;
	// (vel_x, vel_th) --> (leftwheel_vel, rightwheel_vel)
	left_omega = (2*vel - omega * wheelSeparation) / 2 / wheelRadius;
	right_omega = (2*vel + omega * wheelSeparation) / 2 / wheelRadius;

//	ROS_INFO_STREAM("right = " << right << "\t" << "left = " << left << "dr"<< dr);

	// publish to /cmd_wheel_angularVel
	msg.speed1 = left_omega;
	msg.speed2 = right_omega;
	cmd_wheel_angularVel_pub.publish(msg);

	ros::spinOnce();
}

void TwistToMotors::twistCallback(const geometry_msgs::Twist &twist_aux)
{
	ticks_since_target = 0;
	geometry_msgs::Twist twist = twist_aux;
	vel = twist_aux.linear.x;
	omega = twist_aux.angular.z;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"twist2motor");
	TwistToMotors obj;

	obj.spin();

}
