#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

double target_position=0;
double target_duration=0;
double max_range=1.57;

int main(int argc, char** argv){
  ros::init(argc, argv, "sinwave_test");

  ros::NodeHandle n1;
  ros::Publisher neck_pub = n1.advertise<geometry_msgs::Vector3>("/andbot/joint/N0/cmd/targetset", 500);
  geometry_msgs::Vector3 neck;
  neck.x= 1;
  neck.y= 1;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(10.0);
  unsigned int k=0;
  while(n1.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    //neck.x = atof(argv[1]);
    //neck.y = atof(argv[2]);
    neck.x=(double)max_range*sin(0.5*k*0.1);
    ROS_INFO("neck.x %f",neck.x);
    neck_pub.publish(neck);

    last_time = current_time;
    r.sleep();
    k=k+1;
  }
}
