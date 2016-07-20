#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <angelbot/WheelFb.h>
#include <math.h>
#include "odometry_calc.h"


Odometry_calc::Odometry_calc(){

	init_variables(); //Initialize variables used in the node
	ROS_INFO("Started odometry computing node");

	//Subscribing left and right wheel encoder values
	feedback_wheel_angularVel_sub = n.subscribe("/feedback_wheel_angularVel", 10, &Odometry_calc::feedback_wheel_angularVelCallback, this);

	//Creating a publisher for odom
  	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

	//Retrieving parameters of this node
	get_node_params();
}

void Odometry_calc::init_variables()
{
	rate = 10;

	t_delta = ros::Duration(1.0 / rate);
	t_next = ros::Time::now() + t_delta;

	vel = 0.0;
	omega = 0.0;

	left_omega=0.0;
	right_omega=0.0;

	disXdir=0.0,disYdir=0.0,disTheta=0.0;

	current_time = ros::Time::now();
	last_time = ros::Time::now();
}
void Odometry_calc::get_node_params()
{
	if(n.getParam("rate", rate))
	{
		ROS_INFO_STREAM("Rate from param" << rate);
	}

	if(n.getParam("wheelSeparation", wheelSeparation))
	{
		ROS_INFO_STREAM("wheelSeparation from param =" << wheelSeparation);
	}

	if(n.getParam("wheelRadius", wheelRadius))
	{
		ROS_INFO_STREAM("wheelRadius from param =" << wheelRadius);
	}
/*
	ROS_INFO_STREAM("wheelSeparation" << wheelSeparation);
	ROS_INFO_STREAM("wheelRadius=" << wheelRadius);
*/
}
//Spin function
void Odometry_calc::spin()
{
     ros::Rate loop_rate(rate);
     while (ros::ok())
	{
		update();
		loop_rate.sleep();
	}
}
//Update function
void Odometry_calc::update()
{
	//current_time = ros::Time::now();


	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(disTheta);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "angelbot_base";

	odom_trans.transform.translation.x = disXdir;
	odom_trans.transform.translation.y = disYdir;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = disXdir;
	odom.pose.pose.position.y = disYdir;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "angelbot_base";
	odom.twist.twist.linear.x = vel;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = omega;

	//publish the message
	odom_pub.publish(odom);

	ros::spinOnce();
//		ROS_INFO_STREAM("Not in loop");
}
void Odometry_calc::feedback_wheel_angularVelCallback(const angelbot::WheelFb &wheel)
{
	// ROS_INFO_STREAM("Right tick" << right_ticks->data);
  left_omega = wheel.speed1;
  right_omega = wheel.speed2;
  double deltaX,deltaY,deltaTheta,elapsed;

  vel = (right_omega + left_omega) * wheelRadius / 2;
  omega = (right_omega - left_omega) * wheelRadius / wheelSeparation;

  current_time = ros::Time::now();
  elapsed = (current_time - last_time).toSec();
  last_time = current_time;

  ROS_INFO_STREAM("elapsed =" << elapsed);

  deltaX = vel * cos(disTheta) * elapsed;
  deltaY = vel * sin(disTheta) * elapsed;
  deltaTheta = omega * elapsed;

  disXdir +=deltaX;
  disYdir +=deltaY;
  disTheta +=deltaTheta;
}
int main(int argc, char **argv)

{
	ros::init(argc, argv,"odometry_tf");
	Odometry_calc obj;
	obj.spin();

	return 0;
}
