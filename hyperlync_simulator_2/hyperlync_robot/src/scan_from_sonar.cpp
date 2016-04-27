/*
 *
 *  Created Or Edited on: 12/10/2015
 *      Author: Dott. Antonio Mauro Galiano
 *		antoniomauro.galiano@gmail.com
 *
 *   This node creates the usefull scan from the sonar sensors to try to build a map
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>


double distance_scan;
double sonar_1, sonar_2, sonar_3, sonar_4, sonar_5, sonar_6, sonar_7, sonar_8, sonar_9, sonar_10, sonar_11, sonar_12; 
int rotate;
int count = 0;

void sonar1Callback(const sensor_msgs::Range& sonar1){
	sonar_1=sonar1.range;	
}

void sonar2Callback(const sensor_msgs::Range& sonar2){
	sonar_2=sonar2.range;	
}

void sonar3Callback(const sensor_msgs::Range& sonar3){
	sonar_3=sonar3.range;	
}

void sonar4Callback(const sensor_msgs::Range& sonar4){
	sonar_4=sonar4.range;	
}

void sonar5Callback(const sensor_msgs::Range& sonar5){
	sonar_5=sonar5.range;	
}

void sonar6Callback(const sensor_msgs::Range& sonar6){
	sonar_6=sonar6.range;	
}

void sonar7Callback(const sensor_msgs::Range& sonar7){
	sonar_7=sonar7.range;	
}

void sonar8Callback(const sensor_msgs::Range& sonar8){
	sonar_8=sonar8.range;	
}

void sonar9Callback(const sensor_msgs::Range& sonar9){
	sonar_9=sonar9.range;	
}

void sonar10Callback(const sensor_msgs::Range& sonar10){
	sonar_10=sonar10.range;	
}

void sonar11Callback(const sensor_msgs::Range& sonar11){
	sonar_11=sonar11.range;	
}

void sonar12Callback(const sensor_msgs::Range& sonar12){
	sonar_12=sonar12.range;	
}

int main(int argc, char** argv){
 	ros::init(argc, argv, "scan_from_sonar");
	ros::NodeHandle n;
	ros::Time current_time;
	
        std_msgs::Float64 laser_angle;
	unsigned int num_readings = 12;
  	double laser_frequency = 20;
  	double range[num_readings];
	sensor_msgs::LaserScan scan;
	scan.header.stamp = current_time;
	scan.header.frame_id = "sonar_1";
	scan.angle_min = -6.28;
	scan.angle_max = 0.0;
	scan.angle_increment = 6.28 / num_readings;
	scan.time_increment = (1 / laser_frequency) / (num_readings);
	scan.range_min = 0.02;
	scan.range_max = 4;
	scan.ranges.resize(num_readings);
	ros::Publisher sonar_laser_pub = n.advertise<sensor_msgs::LaserScan>("sonar_scan", 50);
	int flag = 0;

	ros::Subscriber sonar_1_subscriber_;
	sonar_1_subscriber_ = n.subscribe("andbot/range_sonar_1", 100, sonar1Callback);
	ros::Subscriber sonar_2_subscriber_;
	sonar_2_subscriber_ = n.subscribe("andbot/range_sonar_2", 100, sonar2Callback);
	ros::Subscriber sonar_3_subscriber_;
	sonar_3_subscriber_ = n.subscribe("andbot/range_sonar_3", 100, sonar3Callback);
	ros::Subscriber sonar_4_subscriber_;
	sonar_4_subscriber_ = n.subscribe("andbot/range_sonar_4", 100, sonar4Callback);
	ros::Subscriber sonar_5_subscriber_;
	sonar_5_subscriber_ = n.subscribe("andbot/range_sonar_5", 100, sonar5Callback);
	ros::Subscriber sonar_6_subscriber_;
	sonar_6_subscriber_ = n.subscribe("andbot/range_sonar_6", 100, sonar6Callback);
	ros::Subscriber sonar_7_subscriber_;
	sonar_7_subscriber_ = n.subscribe("andbot/range_sonar_7", 100, sonar7Callback);
	ros::Subscriber sonar_8_subscriber_;
	sonar_8_subscriber_ = n.subscribe("andbot/range_sonar_8", 100, sonar8Callback);
	ros::Subscriber sonar_9_subscriber_;
	sonar_9_subscriber_ = n.subscribe("andbot/range_sonar_9", 100, sonar9Callback);
	ros::Subscriber sonar_10_subscriber_;
	sonar_10_subscriber_ = n.subscribe("andbot/range_sonar_10", 100, sonar10Callback);
	ros::Subscriber sonar_11_subscriber_;
	sonar_11_subscriber_ = n.subscribe("andbot/range_sonar_11", 100, sonar11Callback);
	ros::Subscriber sonar_12_subscriber_;
	sonar_12_subscriber_ = n.subscribe("andbot/range_sonar_12", 100, sonar12Callback);

  	float angle;

        
  	ros::Rate r(50);
	
  	while(n.ok()){
			ros::Time scan_time = ros::Time::now();
			scan.header.stamp = scan_time;
			scan.ranges[0] = sonar_1;
			scan.ranges[1] = sonar_2;
			scan.ranges[2] = sonar_3;
			scan.ranges[3] = sonar_4;
			scan.ranges[4] = sonar_5;
			scan.ranges[5] = sonar_6;
			scan.ranges[6] = sonar_7;
			scan.ranges[7] = sonar_8;
			scan.ranges[8] = sonar_9;
			scan.ranges[9] = sonar_10;
			scan.ranges[10] = sonar_11;
			scan.ranges[11] = sonar_12;
			sonar_laser_pub.publish(scan);
			ros::spinOnce();
			r.sleep();
		}
  	
}
