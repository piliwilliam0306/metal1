#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "math.h"
#include <iostream>
#define PI 3.14159
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const geometry_msgs::Vector3 msg)
{
  ROS_INFO("I heard x: [%f]", msg.x);
  ROS_INFO("I heard y: [%f]", msg.y);
  ROS_INFO("I heard z: [%f]\n", msg.z);

}

double x=20,y=400,z=-20,th0=0,th1=0;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/andbot/joint/left_arm/goal", 1000, chatterCallback);

  double temp1=pow(x,2)+pow(y,2)+pow(z,2);
  double len_P=pow(temp1,0.5);
  if(x>=0&&y<0)
    th0=atan((double)abs(x)/abs(y));
  else if(x>=0&&y>=0)
      th0=PI-atan((double)abs(x)/abs(y));
    else if(x<0&&y>=0)
        th0=PI+atan((double)abs(x)/abs(y));
      else if(x<0&&y<0)
          th0=2*PI-atan((double)abs(x)/abs(y));


  double temp2=pow(pow(x,2)+pow(y,2),0.5);
  if(z>=0)
    th1=atan((double)abs(z)/temp2);
  else if(z<0)
    th1=-atan((double)abs(z)/temp2);
 
    std::cout<<"x "<<x<<std::endl;
    std::cout<<"y "<<y<<std::endl;
    std::cout<<"z "<<z<<std::endl;
    std::cout<<"len_P "<<len_P<<std::endl;
    std::cout<<"th0 "<<th0<<std::endl;
    std::cout<<"th1 "<<th1<<std::endl;



  ros::spin();

  return 0;
}
