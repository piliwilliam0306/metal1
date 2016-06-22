#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_position_listener_ekf");

  ros::NodeHandle node;

  ros::Publisher robot_position_pub =  node.advertise<geometry_msgs::PoseStamped>("/andbot/current_position", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/map", "/base_footprint",
                              now, ros::Duration(3.0));
      listener.lookupTransform("/map", "/base_footprint",
                             now, transform);

     // listener.lookupTransform("/map", "/base_footprint",  
     //                          ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::PoseStamped andbot_pose;
    andbot_pose.header.frame_id = "map";
    andbot_pose.header.stamp = ros::Time::now();
    andbot_pose.pose.position.x = transform.getOrigin().x();
    andbot_pose.pose.position.y = transform.getOrigin().y();
    andbot_pose.pose.position.z = transform.getOrigin().z();
    andbot_pose.pose.orientation.x = transform.getRotation().x();
    andbot_pose.pose.orientation.y = transform.getRotation().y();
    andbot_pose.pose.orientation.z = transform.getRotation().z();
    andbot_pose.pose.orientation.w = transform.getRotation().w();

    robot_position_pub.publish(andbot_pose);

    rate.sleep();
  }
  return 0;
};
