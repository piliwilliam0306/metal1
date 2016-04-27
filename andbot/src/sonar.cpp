#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <andbot/Sonar.h>

double sonar1 = 0.0;
double sonar2 = 0.0;
double sonar3 = 0.0;
double sonar4 = 0.0;
double sonar5 = 0.0;
double sonar6 = 0.0;
double sonar7 = 0.0;
double sonar8 = 0.0;

double field_of_view = 0.523598776;
double min_range = 0.02;
double max_range = 2.46;

ros::Publisher sonar_pub1;
ros::Publisher sonar_pub2;
ros::Publisher sonar_pub3;
ros::Publisher sonar_pub4;
ros::Publisher sonar_pub5;
ros::Publisher sonar_pub6;
ros::Publisher sonar_pub7;
ros::Publisher sonar_pub8;
ros::Subscriber sonar_sub;
ros::Time current_time;

void sonarCallback(const andbot::Sonar &sonar_msg)
{
  sonar1 = double(sonar_msg.sonar1)/100;
  sonar2 = double(sonar_msg.sonar2)/100;
  sonar3 = double(sonar_msg.sonar3)/100;
  sonar4 = double(sonar_msg.sonar4)/100;
  sonar5 = double(sonar_msg.sonar5)/100;
  sonar6 = double(sonar_msg.sonar6)/100;
  sonar7 = double(sonar_msg.sonar7)/100;
  sonar8 = double(sonar_msg.sonar8)/100;

}

int main(int argc, char** argv){
  ros::init(argc, argv, "sonar");

  ros::NodeHandle n1, n2;
  sonar_pub1 = n1.advertise<sensor_msgs::Range>("/sonar1", 50);
  sonar_pub2 = n1.advertise<sensor_msgs::Range>("/sonar2", 50);
  sonar_pub3 = n1.advertise<sensor_msgs::Range>("/sonar3", 50);
  sonar_pub4 = n1.advertise<sensor_msgs::Range>("/sonar4", 50);
  sonar_pub5 = n1.advertise<sensor_msgs::Range>("/sonar5", 50);
  sonar_pub6 = n1.advertise<sensor_msgs::Range>("/sonar6", 50);
  sonar_pub7 = n1.advertise<sensor_msgs::Range>("/sonar7", 50);
  sonar_pub8 = n1.advertise<sensor_msgs::Range>("/sonar8", 50);

  sonar_sub = n2.subscribe("sonar", 10, sonarCallback);

  ros::Rate r(1.0); // rate for publishing sonar

  //publish the range message over ROS
  sensor_msgs::Range range_msg;
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.field_of_view = field_of_view;
  range_msg.min_range = min_range;
  range_msg.max_range = max_range;

  while(n1.ok()){

    ros::spinOnce();               // check for incoming messages

    range_msg.range = sonar1;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id =  "/sonar_1";
    sonar_pub1.publish(range_msg);

    range_msg.range = sonar2;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id =  "/sonar_2";
    sonar_pub2.publish(range_msg);

    range_msg.range = sonar3;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id =  "/sonar_3";
    sonar_pub3.publish(range_msg);

    range_msg.range = sonar4;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id =  "/sonar_4";
    sonar_pub4.publish(range_msg);

    range_msg.range = sonar5;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id =  "/sonar_5";
    sonar_pub5.publish(range_msg);

    range_msg.range = sonar6;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id =  "/sonar_6";
    sonar_pub6.publish(range_msg);

    range_msg.range = sonar7;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id =  "/sonar_7";
    sonar_pub7.publish(range_msg);

    range_msg.range = sonar4;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.frame_id =  "/sonar_8";
    sonar_pub8.publish(range_msg);

    r.sleep();
  }
}
