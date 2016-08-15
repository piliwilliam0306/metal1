#include "ros/ros.h"
#include <iostream>
#include "andbot1dot2/DriverState.h"
#include <iostream>
#include <sstream>

using namespace std;

int main(int argc, char **argv) 
{
	ros::init(argc, argv, "demo_service_client");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	ros::ServiceClient client = n.serviceClient<andbot1dot2::DriverState>("demo_service");
	while (ros::ok()) 
	{
		andbot1dot2::DriverState srv;
		std::stringstream ss;
		ss << "Sending from Here";
		srv.request.in = ss.str();
		if (client.call(srv)) 
		{
			ROS_INFO("From Client [%s], Server says [%s]",srv.request.in.c_ str(),srv.response.out.c_str());
		}
		else 
		{
			ROS_ERROR("Failed to call service");
			return 1;
		}
		ros::spinOnce();
		loop_rate.sleep();
	} 
	return 0;
}