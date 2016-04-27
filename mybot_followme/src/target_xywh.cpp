#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float32MultiArray.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/image_encodings.h>
#include "opencv2/objdetect/objdetect.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class Target_XYWH
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher target_xywh_pub_;
  ros::Publisher cmd_pub_;

  std_msgs::Float32MultiArray brray;


  unsigned long  start_time, now;

  struct timeval tp;
  double before_filter_x;
  double before_filter_y;
  double filter_x = 0.0;
  double filter_y = 0.0;
  double filter_width = 0.0;
  double filter_height = 0.0;
  //int firstBodyFound = 0;

  CascadeClassifier face_cascade;
//  String face_cascade_name = "/home/odroid/catkin_ws/src/mybot_followme/res/haarcascades/haarcascade_frontalface_default.xml";

  String face_cascade_name = "/home/odroid/catkin_ws/src/mybot_followme/res/haarcascades/haarcascade_upperbody.xml";

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
  Target_XYWH()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, 
      &Target_XYWH::imageCb, this);
    image_pub_ = it_.advertise("/target_xywh/output_video", 1);
    target_xywh_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/target_xywh", 1000);

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/andbot/cmd_vel", 1);


//    std_msgs::Float32MultiArray array;
    brray.data.clear();
    for (int i=0; i<4; i++)
        brray.data.push_back(0.0);
    
    
//    cv::namedWindow(OPENCV_WINDOW);

    //Load the cascades
    if (!face_cascade.load(face_cascade_name))  { 
        printf("--(!)Error loading face cascade\n"); 
    };

    
    gettimeofday(&tp, NULL);
    start_time = tp.tv_sec * 1000000 + tp.tv_usec;

  }

  ~Target_XYWH()
  {
//    cv::destroyWindow(OPENCV_WINDOW);
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

    std::vector<Rect> faces;
    Mat frame_gray;
    Mat frame(cv_ptr->image);
    resize(frame, frame, Size(160, 120));
    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //t = (double)cvGetTickCount();
    //-- Detect faces
    //face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(16, 12), Size(80, 60) ); // 30ms

    face_cascade.detectMultiScale( frame_gray, faces, 1.05, 2); // 35ms
    
    //t = (double)cvGetTickCount() - t;
    //printf( "detection time = %g ms\n", t/((double)cvGetTickFrequency()*1000.) );

    if (faces.size() > 0) {
        //myFilter((faces[0].x + faces[0].width/2) * 4 - 320, (faces[0].y + faces[0].height/2) * 4 - 240);
        //firstBodyFound = 1;
        before_filter_x = (faces[0].x + faces[0].width/2) * 4 - 320;
        before_filter_y = (faces[0].y + faces[0].height/2) * 4 - 240;
    }
    else
    {
        //myFilter(0.0, 0.0);
        before_filter_x = 0.0;
        before_filter_y = 0.0;
    }

    myFilter(before_filter_x, before_filter_y);

    gettimeofday(&tp, NULL);
    now = tp.tv_sec * 1000000 + tp.tv_usec; 
    double t = (double) (now - start_time) / 1000000;

    printf("%f    %f    %f\n", t, before_filter_x, filter_x);

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

        // publishing
        brray.data[0] = 0.0; // x: 0.0 (depth)
        brray.data[1] = -filter_x; // y: -320 ~ +320
        brray.data[2] = -filter_y; // z: -240 ~ +240
        brray.data[3] = 0.0;
        brray.data[4] = 0.0;

        target_xywh_pub_.publish(brray);

// control base to head to the target
        geometry_msgs::TwistPtr cmd(new geometry_msgs::Twist());
	cmd->angular.z = -brray.data[1]/320 * 1.5; 
        cmd_pub_.publish(cmd);
    }

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    //cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_xywh");
  Target_XYWH ic;
  ros::spin();
  return 0;
}
