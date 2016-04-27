#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "../include/filt.h"

double raw_data=0.0;
double filter_data=0.0;
std_msgs::Float32 filter_data_msgs;
ros::Publisher output_pub;  
ros::Subscriber input_sub;
Filter *my_filter;


void FIR_Filter(){

  filter_data =(my_filter->do_sample(raw_data))-8.0;

}

void FIR_FilterCallback(const std_msgs::Float32& msg){
   raw_data=msg.data;
   FIR_Filter();  
   filter_data_msgs.data=filter_data;
   output_pub.publish(filter_data_msgs);
   ROS_INFO("PreValue:%f  filterValue %f",raw_data,filter_data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fir_noisefilter");
  ros::NodeHandle nh_;
  output_pub = nh_.advertise<std_msgs::Float32>("/andbot/LidarLite/filter_Distance", 500);
  input_sub = nh_.subscribe("/andbot/LidarLite/Distance", 1000, FIR_FilterCallback);
  my_filter = new Filter(LPF,10,10,0.8);


  ros::Rate r(100);
  while(nh_.ok())
  {
    ros::spinOnce();
    r.sleep();

  }

  return(0);
}

