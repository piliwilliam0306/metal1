#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sensor_msgs/Joy.h>

#define KEYCODE_R 0x43 
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

#define KEYCODE_P 'p'   // for stop the robot
#define KEYCODE_A 'a'   // for starting laser scan
#define KEYCODE_S 's'   // for stopping laser scan

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  ros::Publisher joy_pub_;
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(0.3),
  a_scale_(0.3)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("andbot/cmd_vel", 1);
  joy_pub_ =   nh_.advertise<sensor_msgs::Joy>("joy", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  
  return(0);
}


void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;
  bool dirty1=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);

    sensor_msgs::Joy joy;
    joy.buttons.resize(1);
    joy.buttons[0] = 0;
  
    switch(c)
    {
      case KEYCODE_L:
        ROS_INFO("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_INFO("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_INFO("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_INFO("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_P:
        ROS_INFO("STOP MOVING");
        linear_ = 0.0;
        angular_ = 0.0;
        dirty = true;
        break;

      case KEYCODE_A:
        ROS_INFO("A");
        joy.buttons[0] = 1;
        joy_pub_.publish(joy);
        break;

      case KEYCODE_S:   // stop scanning
        ROS_INFO("S");
        joy.buttons[0] = 0;
        joy_pub_.publish(joy);
        break;

    }
   

    geometry_msgs::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;

    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }

//    if (dirty1 == true)
//    {
//      joy_pub_.publish(joy);
//      dirty1=false;
//    }
  }


  return;
}

