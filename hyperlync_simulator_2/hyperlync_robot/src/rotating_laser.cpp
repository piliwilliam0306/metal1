/*
 *
 *  Created Or Edited on: xx/09/2015
 *      Author: Dott. Antonio Mauro Galiano
 *		antoniomauro.galiano@gmail.com
 *      Author: Kevin Kuei
 *		kevin.kuei.0321@gmail.com
 *   
 *   ATTENTION: "scan" topic is the one related to the single beam
 *              "laser" topic is the one related to the 180° laser message 
 */

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>


double distance_scan;
int rotate;
int count = 0;

void laserCallback(const sensor_msgs::LaserScan& laserscan){
	distance_scan = laserscan.ranges[1];	
}

void joyCallback(const sensor_msgs::JoyConstPtr& joy){
	rotate=joy->buttons[0];
	ROS_INFO("ROTATE %d",rotate);
}

int main(int argc, char** argv){
 	ros::init(argc, argv, "rotating_laser");
	ros::NodeHandle n;
	ros::Time current_time;
	
        std_msgs::Float64 laser_angle;
	unsigned int num_readings = 360;
  	double laser_frequency = 10;
  	double range[num_readings];
	sensor_msgs::LaserScan scan;
	scan.header.stamp = current_time;
	scan.header.frame_id = "laser";
	scan.angle_min = -3.14;
	scan.angle_max = 0.0;
	scan.angle_increment = 3.14 / num_readings;
	scan.time_increment = (1 / laser_frequency) / (num_readings);
	scan.range_min = 0.02;
	scan.range_max = 30.0;
	scan.ranges.resize(num_readings);
	ros::Publisher laser_pub = n.advertise<sensor_msgs::LaserScan>("laser", 50);
	int flag = 0;

  	
 	ros::Publisher laser_angle_pub = n.advertise<std_msgs::Float64>("/andbot/laser_position_controller/command", 50);
	ros::Subscriber laser_subscriber_;
	laser_subscriber_ = n.subscribe("scan", 100, laserCallback);

	ros::Subscriber joy_subscriber_;
	joy_subscriber_ = n.subscribe("joy", 100, joyCallback);

  	float angle;

        angle = 0;

        ros::Rate r1(50);
        while(n.ok()) {
            angle -= 6.28/720;
            laser_angle.data = angle;
            laser_angle_pub.publish(laser_angle);
            
            r1.sleep();
            //ros::spinOnce(); 
            if (angle <= -1.57) break;
        }

        sleep(5); // to make sure the laser joint move to home position

        angle = -1.57;
  	ros::Rate r(50);
	//ROS_INFO("USCITO DA HOME STEP");
	//std_msgs::Float64 laser_angle;
  	while(n.ok()){
		if (rotate == 1){
			ROS_INFO("count %d",count);
			ROS_INFO("ANGLE %f",angle);

		        current_time = ros::Time::now();
		        laser_angle.data = angle;
			laser_angle_pub.publish(laser_angle);   // set laser joint position
		        //ros::spinOnce();

			scan.header.stamp = current_time;
			scan.ranges[count] = distance_scan;		

			if (count>=359) {
		        	laser_pub.publish(scan);  // publish Laserscan message
				while(n.ok()) {
				    angle -= 6.28/720;
				    laser_angle.data = angle;
				    laser_angle_pub.publish(laser_angle);
				    
				    r1.sleep();
				    //ros::spinOnce(); 
				    if (angle <= -1.57) break;
				}				
				//rotate=0;
				//
				angle = -1.57;
				count=0;
				sleep(5);
				ROS_INFO("count inside %d",count);
				ROS_INFO("ANGLE inside %f",angle);
		        }

		        if (count>=359) flag = 1;
		        else if (count<=0) flag = 0;
		        if (flag == 0){
		                angle  = angle + 0.008722222;
		                count++;
		        }
		        else if (flag == 1){
		                angle  = angle - 0.008722222;
		                count--;
		        }
		}
			ros::spinOnce();
			r.sleep();
		}
  	
}
