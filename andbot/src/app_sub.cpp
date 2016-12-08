#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include "actionlib_msgs/GoalID.h"
#include <stdlib.h>

using namespace std;

void SlamMode()
{
	system("rosnode kill joint_state_publisher");
	system("rosnode kill robot_state_publisher");
	system("rosnode kill move_base");
	system("rosnode kill neato_laser_publisher");
	system("rosnode kill mybot_xv11_angular_bound_filter");
	system("rosnode kill mybot_base_controller_v1");
	system("rosnode kill mybot_stop_when_goal_reached");
	system("rosnode kill serial_node");
	system("rosnode kill mybot_odometry_v1");
	system("rosnode kill slam_gmapping");
	system("rosnode kill serial_node1");
	system("rosnode kill serial_node2");
	system("rosnode kill serial_node3");
	system("rosnode kill serial_node4");
	system("rosnode kill serial_node5");
	system("rosnode kill serial_node6");
	system("rosnode kill left_arm_moveR");
	system("rosnode kill right_arm_moveR");
	system("rosnode kill action");
	system("launch andbot 35_5F_gmapping.launch");
}

void NavMode()
{
	system("roscd andbot/map");
	system("rosrun map_server map_saver -f mappp");

	system("rosnode kill sonar");
	system("rosnode kill position_listener_ekf");
	system("rosnode kill odom_ekf");
	system("rosnode kill mpu6050");
	system("rosnode kill robot_pose_ekf");
	system("rosnode kill serial_node1");
	system("rosnode kill serial_node2");
	system("rosnode kill slam_gmapping");
	system("rosnode kill mybot_odom_ekf_v1");
	system("rosnode kill serial_node");
	system("rosnode kill mybot_base_controller_v1");
	system("rosnode kill mybot_rplidar_laser_angular_bound_filter");
	system("roslaunch andbot andbot_v1.launch");
}

void Reboot()
{
	system("echo odroid | sudo -S reboot");
}

void Shutdown()
{
	system("echo odroid | sudo -S sync;sync;shutdown -h now");
}

void CancelGoal()
{

	system("rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}");
/*  cout << "cancel goal" << endl;
//  system("rostopic pub /move_base/cancel actionlib_msgs/GoalID -- {}");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1000);
  actionlib_msgs::GoalID goal_id;
  if (ros::ok())
	{
  	cout << "ros is ok" << endl;
	chatter_pub.publish(goal_id);
	ros::spinOnce();
	}
  cout << "exit cancel goal" << endl;*/
}


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  string tmp = msg->data;
  if(tmp.compare("1")==0)
	{
		cout << "SLAM MODE" << endl;
	}
  else if(tmp.compare("2")==0)
	{
        cout << "NAV MODE" << endl;
	}
  else if(tmp.compare("3")==0)
	{
        Reboot();
	}
  else if(tmp.compare("4")==0)
	{
        Shutdown();
	}
  else if(tmp.compare("5")==0)
	{
        CancelGoal();
	}
  else
	{
	ROS_INFO("ERROR");
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "app_sub");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("robot_utilities", 1000, chatterCallback);
  ros::spin();

  return 0;
}
