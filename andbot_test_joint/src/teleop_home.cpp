#include <ros/ros.h>
#include <std_msgs/Float64.h>
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

class TeleopHome
{
public:
  TeleopHome();
  TeleopHome(int,double);
  void keyLoop();
  int mode;
  double speed;
private:
  ros::NodeHandle nh_;
  double angular_0,angular_1;
  double angular_increment_0,angular_increment_1;
  ros::Publisher offset_pub_0,offset_pub_1;
 
};

TeleopHome::TeleopHome():
  mode(0),angular_0(0),angular_1(0),angular_increment_0(0.005),angular_increment_1(0.005)
{
if(mode==0){
  offset_pub_0 = nh_.advertise<std_msgs::Float64>("andbot/joint/L0/cmd/offset", 1);
  offset_pub_1 = nh_.advertise<std_msgs::Float64>("andbot/joint/L1/cmd/offset", 1);
}
}

TeleopHome::TeleopHome(int m,double s):
  angular_0(0),angular_1(0),angular_increment_0(0.005),angular_increment_1(0.005),speed(s),mode(m)
{
if(mode==0){
  offset_pub_0 = nh_.advertise<std_msgs::Float64>("andbot/joint/L0/cmd/offset_v", 1);
  offset_pub_1 = nh_.advertise<std_msgs::Float64>("andbot/joint/L1/cmd/offset_v", 1);
}
if(mode==1){
  offset_pub_0 = nh_.advertise<std_msgs::Float64>("andbot/joint/L3/cmd/offset_v", 1);
  offset_pub_1 = nh_.advertise<std_msgs::Float64>("andbot/joint/L2/cmd/offset_v", 1);
}
if(mode==2){
  offset_pub_0 = nh_.advertise<std_msgs::Float64>("andbot/joint/R0/cmd/offset_v", 1);
  offset_pub_1 = nh_.advertise<std_msgs::Float64>("andbot/joint/R1/cmd/offset_v", 1);
}
if(mode==3){
  offset_pub_0 = nh_.advertise<std_msgs::Float64>("andbot/joint/R3/cmd/offset_v", 1);
  offset_pub_1 = nh_.advertise<std_msgs::Float64>("andbot/joint/R2/cmd/offset_v", 1);
}

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
  ros::init(argc, argv, "teleop_home");
  int MODE=atoi(argv[1]);
  double SPEED=atof(argv[2]);
  TeleopHome teleop_home(MODE,SPEED);
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  signal(SIGINT,quit);

  teleop_home.keyLoop();
  
  return(0);
}


void TeleopHome::keyLoop()
{
  char c;

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
  puts("Use arrow keys to move the joint offset.");

  ros::Rate r(50.0);

  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_U:
        //angular_0=angular_0+angular_increment_0;
	angular_0=speed;
	ROS_INFO("UP %f",angular_0);
        break;
      case KEYCODE_D:
        //angular_0=angular_0-angular_increment_0;
	angular_0=-1*speed;
	ROS_INFO("DOWN %f",angular_0);
        break;
      case KEYCODE_R:
        //angular_1=angular_1-angular_increment_1;
	angular_1=-1*speed;
	ROS_INFO("RIGHT %f",angular_1);
        break;
      case KEYCODE_L:
        //angular_1=angular_1+angular_increment_1;
	angular_1=speed;
	ROS_INFO("LEFT %f",angular_1);
        break;
      case KEYCODE_P:
        ROS_INFO("STOP MOVING");
        angular_0 = 0.0;
	angular_1 = 0.0;
        break;


    }

    std_msgs::Float64 offset_0,offset_1;
    offset_0.data = angular_0;
    offset_1.data = angular_1;

    offset_pub_0.publish(offset_0);
    offset_pub_1.publish(offset_1);

  }


  return;
}
