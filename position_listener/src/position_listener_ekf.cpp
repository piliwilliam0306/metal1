#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_position_listener_ekf");

  ros::NodeHandle node;

  ros::Publisher robot_position_pub =  node.advertise<geometry_msgs::PoseStamped>("/angelbot/current_position", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/map", "/angelbot_base",
                              now, ros::Duration(3.0));
      listener.lookupTransform("/map", "/angelbot_base",
                             now, transform);

     // listener.lookupTransform("/map", "/base_footprint",  
     //                          ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::PoseStamped angelbot_pose;
    angelbot_pose.header.frame_id = "map";
    angelbot_pose.header.stamp = ros::Time::now();
    angelbot_pose.pose.position.x = transform.getOrigin().x();
    angelbot_pose.pose.position.y = transform.getOrigin().y();
    angelbot_pose.pose.position.z = transform.getOrigin().z();
    angelbot_pose.pose.orientation.x = transform.getRotation().x();
    angelbot_pose.pose.orientation.y = transform.getRotation().y();
    angelbot_pose.pose.orientation.z = transform.getRotation().z();
    angelbot_pose.pose.orientation.w = transform.getRotation().w();

    robot_position_pub.publish(angelbot_pose);

    rate.sleep();
  }
  return 0;
};
