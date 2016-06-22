
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <math.h>

#define PI 3.14159265
double i = 0.0;
double count1 = 0.0;

ros::Publisher chatter_pub1;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "acc_lim_test");

  ros::NodeHandle n;

  chatter_pub1 = n.advertise<geometry_msgs::Vector3>("cmd_wheel_angularVel", 1000);

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
	geometry_msgs::Vector3 vector3;
	for (int i = 0; i < 360; i++)
		{
		    vector3.x=count1;
		    vector3.y=count1;
		    chatter_pub1.publish(vector3);
		    loop_rate.sleep();
		    count1=3*sin(i*PI/180);
		}
  }

  return 0;
}
