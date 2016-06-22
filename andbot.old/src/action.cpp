#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <stdio.h>
#include <stdlib.h>
#define PI 3.14159

unsigned int action_state=0;
unsigned int move_state=0;

struct Arm_Pose{
  double x;
  double y;
  double z;
  double fi;
  double t;
};

geometry_msgs::Pose pose_left,pose_right;
unsigned int current_time=0, last_time,duration_time=0;


Arm_Pose phome=
{0,-485,0,0,5};

struct KunFu{
  Arm_Pose left_p0,right_p0;
  Arm_Pose left_p1,right_p1;

}kunfu=
{
//  {0,-350,0,0,0},{0,-350,0,0,0},
//  {485,0,0,0,0},{485,0,0,0,0},
  {200,100,0,0,3},{200,100,0,0,3},
  {450,100,0,0,3},{450,100,0,0,3},

};

struct HoldTray{
  Arm_Pose left_p0,right_p0;
  Arm_Pose left_p1,right_p1;
}holdtray=
{
  {0,-350,0,0,0},{0,-350,0,0,0},
  {400,-200,0,0,0},{400,-200,0,0,0},
};

struct Strong{
  Arm_Pose left_p0a,right_p0a;
  Arm_Pose left_p0b,right_p0b;
  Arm_Pose left_p1a,right_p1a;
  Arm_Pose left_p1b,right_p1b;

}strong=
{
  {0,-350,0,0,0},{0,-350,0,0,0},
  {0,-350,0,0.7854,0},{0,-350,0,-0.7854,0},
  {220,180,-200,0.2854,0},{220,180,200,-0.2854,0},
  {220,180,-200,1.0,0},{220,180,200,-1.0,0},

//  {200,200,-200,2.04,0},{200,200,200,1.1,0},
};

 void predefineCallback(const std_msgs::UInt8& msg){
   action_state=msg.data;
   move_state=0;
   duration_time=0;

}


void move_to_pose_left(const Arm_Pose& pose)
{
  pose_left.position.x =pose.x; 
  pose_left.position.y =pose.y; 
  pose_left.position.z =pose.z; 
  pose_left.orientation.x =pose.fi; 
  pose_left.orientation.y =pose.t; 

}
void move_to_pose_right(const Arm_Pose& pose)
{
  pose_right.position.x =pose.x; 
  pose_right.position.y =pose.y; 
  pose_right.position.z =pose.z; 
  pose_right.orientation.x =pose.fi; 
  pose_right.orientation.y =pose.t; 
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "action");
  ros::NodeHandle nh_;
  ros::Publisher action_left_pub = nh_.advertise<geometry_msgs::Pose>("/andbot/left_arm/goal", 500);
  ros::Publisher action_right_pub = nh_.advertise<geometry_msgs::Pose>("/andbot/right_arm/goal", 500);
  ros::Subscriber pose_sub = nh_.subscribe("/andbot/predefinedPoses", 1000, predefineCallback);

  current_time = ros::Time::now().toSec();
  last_time = ros::Time::now().toSec();
  ros::Rate r(1);
  while(nh_.ok())
  {
    current_time = ros::Time::now().toSec();
    duration_time=duration_time+(current_time-last_time);
    last_time = ros::Time::now().toSec();

    ros::spinOnce();
    switch(action_state)
    {
	case 0:
            ROS_INFO("HOME(LIMP) t=%d",duration_time);
            move_to_pose_left(phome);
            move_to_pose_right(phome);
	    break;

	case 1:
 	  {
            ROS_INFO("KUN_FU t=%d",duration_time);
	    switch(move_state)
            {
              case 0:
                move_to_pose_left(kunfu.left_p0);
                move_to_pose_right(kunfu.right_p0);
                ROS_INFO("l:p0 r:p0");
                if(duration_time>=10) {move_state=1;duration_time=0;}
                break;
              case 1:
                move_to_pose_left(kunfu.left_p0);
                move_to_pose_right(kunfu.right_p1);
                ROS_INFO("l:p0 r:p1");
                if(duration_time>=5) {move_state=2;duration_time=0;}
                break;
              case 2:
                move_to_pose_left(kunfu.left_p1);
                move_to_pose_right(kunfu.right_p0);
                ROS_INFO("l:p1 r:p0");
                if(duration_time>=5) {move_state=1;duration_time=0;}
                break;


            }
	    break;

          }
	case 2:
 	  {
            ROS_INFO("HOLD_TRAY t=%d",duration_time);
	    switch(move_state)
            {

              case 0:
                move_to_pose_left(holdtray.left_p0);
                move_to_pose_right(holdtray.right_p0);
                ROS_INFO("l:p0 r:p0");
		if(duration_time>=10) {move_state=1;}
		break;
              case 1:
                move_to_pose_left(holdtray.left_p1);
                move_to_pose_right(holdtray.right_p1);
                ROS_INFO("l:p1 r:p1");
		break;

            }
	    break;

          }
	case 3:
 	  {
            ROS_INFO("Strong t=%d",duration_time);
	    switch(move_state)
            {

              case 0:
                move_to_pose_left(strong.left_p0a);
                move_to_pose_right(strong.right_p0a);
                ROS_INFO("l:p0a r:p0a");
		if(duration_time>=10) {move_state=1;duration_time=0;}
		break;
              case 1:
                move_to_pose_left(strong.left_p1a);
                move_to_pose_right(strong.right_p1a);
                ROS_INFO("l:p1a r:p1a");
		if(duration_time>=15) {move_state=0;duration_time=0;}
		break;

            }
	    break;

          }

    }

    action_left_pub.publish(pose_left);
    action_right_pub.publish(pose_right);


    r.sleep();
  }


  return(0);
}
