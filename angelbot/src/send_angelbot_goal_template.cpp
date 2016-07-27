#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <iostream>

//Declaring a new SimpleActionClient with action of move_base_msgs::MoveBaseAction
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "navigation_goals");
	
	//Initiating move_base client
	MoveBaseClient ac("move_base", true);
	
	//Waiting for server to start
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server");
	}

	//Declaring move base goal
	move_base_msgs::MoveBaseGoal goal;
	
	//Setting target frame id and time in the goal action
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	
	//Retrieving pose from command line other vice execute a default value
	try{
		goal.target_pose.pose.position.x = atof(argv[1]);
		goal.target_pose.pose.position.y = atof(argv[2]);
		goal.target_pose.pose.orientation.w = atof(argv[3]);
	   }
	catch(int e){


		goal.target_pose.pose.position.x = 1.0;
		goal.target_pose.pose.position.y = 1.0;
		goal.target_pose.pose.orientation.w = 1.0;


	}
	ROS_INFO("Sending move base goal");
	
	//Sending goal
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		ROS_INFO("Robot has arrived to the goal position");
	else{
		ROS_INFO("The base failed for some reason");
	}
	return 0;
}