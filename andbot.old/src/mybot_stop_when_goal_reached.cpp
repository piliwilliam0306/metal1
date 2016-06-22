#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "actionlib_msgs/GoalStatus.h"
#include "actionlib_msgs/GoalStatusArray.h"


int flag = 0;
ros::Publisher cmd_pub_;

void statusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status)
{
  //ROS_INFO("I heard: [%s]", msg->data.c_str());
  if (!status->status_list.empty()) {
     actionlib_msgs::GoalStatus goalStatus = status->status_list[0];
     //testing_pub.publish(goalStatus);

     if (goalStatus.status == 3) {
       if (flag != 1) {
          // stop code is here
          usleep(200000);
          cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
          usleep(200000);
          cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
          usleep(200000);
          cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));

          ROS_INFO("[KKUEI] sent cmd_vel(0, 0, 0)");
          flag = 1;
       }

     } else {
       flag = 0;
     }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber status_sub_ = n.subscribe("/move_base/status", 1000, statusCallback);
  cmd_pub_ = n.advertise<geometry_msgs::Twist>("/andbot/cmd_vel", 1);

  ros::spin();

  return 0;
}
