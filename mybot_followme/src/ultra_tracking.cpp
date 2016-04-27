#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

class Ultra_Tracking
{
  public:
  ros::NodeHandle nh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber cmd_sub_;
  Ultra_Tracking()
  {
    cmd_sub_ = nh_.subscribe("/andbot/LidarLite/filter_Distance", 1 ,&Ultra_Tracking::DistCB,this);

   cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/andbot/cmd_vel", 1);

  }

 void DistCB(const std_msgs::Float32& msg){
  geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
  cmd->linear.x = (msg.data - 60)*0.005 ;
  cmd->angular.z = 0;
  cmd_pub_.publish(cmd);

//   ROS_INFO("SensorValue %f",msg.data);
}


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ultra_tracking");
  Ultra_Tracking ut;
  ros::spin();
  return 0;
}
