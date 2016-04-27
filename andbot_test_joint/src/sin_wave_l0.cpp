#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

double target_position=0;
double target_duration=0;
double max_range=1.57;

int main(int argc, char** argv){
  ros::init(argc, argv, "sinwave_L0_test");

  ros::NodeHandle n1;
  ros::Publisher l0_pub = n1.advertise<geometry_msgs::Vector3>("/andbot/joint/L0/cmd/position", 500);
  geometry_msgs::Vector3 l0;
  l0.x= 0;
  l0.y= 10;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20.0);
  unsigned int k=0;
  while(n1.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    l0.x=(double)max_range*sin(0.02*k*0.2)+1.57;
    ROS_INFO("l0.x %f",l0.x);
    l0_pub.publish(l0);

    last_time = current_time;
    r.sleep();
    k=k+1;
  }
}
