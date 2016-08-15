#include "ros/ros.h"
#include "andbot1dot2/DriverState.h"
#include <iostream>
#include <sstream>

using namespace std;

bool DriverState_service_callback(andbot1dot2::DriverState::Request &req, andbot1dot2::DriverState::Response &res)
{
	std::stringstream ss;
	ss << "Received Here";
	res.out = ss.str();
	ROS_INFO("From Client [%s], Server says [%s]",req.in.c_str(),res. out.c_str());
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "DriverState_service_server");
	ros::NodeHandle n;
	ros::ServiceServer service = n.advertiseService("DriverState_service", DriverState_service_callback);
	ROS_INFO("Ready to receive from client.");
	ros::spin();
	return 0;
}

