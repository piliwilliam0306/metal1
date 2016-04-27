#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float32MultiArray.h"
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Target_XYZ
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Subscriber cloud_sub_;
  ros::Subscriber followMeCmd_sub_;
  ros::Publisher target_xyz_pub_;
  ros::Publisher cmd_pub_;

  std_msgs::Float32MultiArray brray;

  unsigned int action_state=0;

  unsigned long  start_time, now;

  struct timeval tp;
  double before_filter_x;
  double before_filter_y;
  double before_filter_z;
  double filter_x = 0.0;
  double filter_y = 0.0;
  double filter_z = 0.0;
  //int firstBodyFound = 0;

  double min_y_ = -0.3; /**< The minimum y position of the points in the box. */
  double max_y_ = 0.3; /**< The maximum y position of the points in the box. */
  double min_x_ = -0.3; /**< The minimum x position of the points in the box. */
  double max_x_ = 0.3; /**< The maximum x position of the points in the box. */
  double max_z_ = 1.3; /**< The maximum z position of the points in the box. */
  double goal_z_ = 0.6; /**< The distance away from the robot to hold the centroid */
  double z_scale_ = 0.5; /**< The scaling factor for translational robot speed */
  double x_scale_ = 2.0; /**< The scaling factor for rotational robot speed */
  
//  String face_cascade_name = "/home/odroid/catkin_ws/src/mybot_followme/res/haarcascades/haarcascade_frontalface_default.xml";

  // my simple filter
  void myFilter(double x, double y)
  {
    double Kp = 0.6;
    double d_x = x - filter_x;
    double d_y = y - filter_y;

    filter_x += Kp * d_x;
    filter_y += Kp * d_y;
  }

  
public:
  Target_XYZ()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
//    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &Target_XYWH::imageCb, this);


//    image_pub_ = it_.advertise("/target_xywh/output_video", 1);
    target_xyz_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/targetXYZ", 1000);


    cloud_sub_= nh_.subscribe<PointCloud>("/camera/depth/points", 1, &Target_XYZ::cloudCb, this);
    followMeCmd_sub_= nh_.subscribe("/andbot/followMe/cmd", 1, &Target_XYZ::followMeCmdCb, this);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/andbot/cmd_vel", 1);


//    std_msgs::Float32MultiArray array;
    brray.data.clear();
    for (int i=0; i<4; i++)
        brray.data.push_back(0.0);
    
    
    gettimeofday(&tp, NULL);
    start_time = tp.tv_sec * 1000000 + tp.tv_usec;

  }

  ~Target_XYZ()
  {
//    cv::destroyWindow(OPENCV_WINDOW);
  }

  void followMeCmdCb(const std_msgs::UInt8& msg)
  {
    // action_state: 0 (stop follow me)
    // action_state: 1 (start follow me)
    action_state=msg.data;
    ROS_INFO("followMeCmdCb(0:stop  1:start): I got %d ", action_state);

    if (action_state == 0) {
      usleep(500000);
      cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      usleep(500000);
      cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      usleep(500000);
      cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      usleep(500000);
      cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      usleep(500000);
      cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      usleep(500000);
      cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));

    }


  }

  void cloudCb(const PointCloud::ConstPtr&  cloud)
  {
    if (action_state == 0)  return;

    //X,Y,Z of the centroid
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    float z1 = 0.0;
    //Number of points observed
    unsigned int n = 0;
    //Iterate through all the points in the region and find the average of the position
    BOOST_FOREACH (const pcl::PointXYZ& pt, cloud->points)
    {
      //First, ensure that the point's position is valid. This must be done in a seperate
      //if because we do not want to perform comparison on a nan value.
      if (!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
      {
        //Test to ensure the point is within the aceptable box.
        if (pt.y > min_y_ && pt.y < max_y_ && pt.x < max_x_ && pt.x > min_x_ && pt.z < max_z_)
        {
          //Add the point to the totals
          x += pt.x;
          y += pt.y;
          z = std::min(z, pt.z);
          z1 += pt.z;
          n++;
        }
      }
    }

    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    if (n>400)
    {
      x /= n;
      y /= n;
      z1 /= n;
      if(z > max_z_){
        ROS_DEBUG("No valid points detected, stopping the robot");
        /*
        if (enabled_)
        {
          cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
        }
        */
        return;
      }

      ROS_DEBUG("Centroid at %f %f %f with %d points", x, y, z, n);
      publishXYZ(x, y, z);

      if (1)
      {
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
        cmd->linear.x = (z - goal_z_) * z_scale_;
        //cmd->angular.z = -x * x_scale_;
        cmd->angular.z = -1.0 * atan(x/z) * x_scale_;
        cmd_pub_.publish(cmd);
      }
    }
    else
    {
      ROS_DEBUG("No points detected, stopping the robot");
      publishXYZ(0, 0, 0);

      if (1)
      {
        cmd_pub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
      }
    }

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Now, the frame is in "cv_ptr->image" 

    if (1) {
        size_t i = 0;
        Point center(filter_x + 320.0, filter_y + 240);
        cv::circle(cv_ptr->image, center, 50, CV_RGB(0,255,255), 5.0);

        /*
        rectangle(cv_ptr->image,
           Point(filter_x, filter_y),
           Point(filter_x + filter_width, filter_y + filter_height),
           Scalar(0, 255, 255),
           5,
           8);

        */

// control base to head to the target
/*
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
	cmd->angular.z = -brray.data[1]/320 * 1.5; 
        cmd_pub_.publish(cmd);
*/
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }

  void publishXYZ(double x, double y, double z) 
  {
    // publishing
    brray.data[0] = x;
    brray.data[1] = y;
    brray.data[2] = z;
    brray.data[3] = 0.0;
    brray.data[4] = 0.0;

    target_xyz_pub_.publish(brray);
  }

  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mybot_followme");
  Target_XYZ ic;
  ros::spin();
  return 0;
}
