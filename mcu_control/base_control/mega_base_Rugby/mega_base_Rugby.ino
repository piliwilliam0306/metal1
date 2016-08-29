//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

#define LOOPTIME        100
#define RS485Enable1 8
#define RS485Enable2 9
#define RS485Transmit    HIGH
#define RS485Receive     LOW 

#define slave1 1
#define slave2 2
#define slave3 3
#define slave4 4

double omega_left_target = 0.0;
double omega_right_target = 0.0;
double omega_left_actual = 0;
double omega_right_actual = 0;
double omega_pan_target = 0.0;
double omega_tilt_target = 0.0;
double omega_pan_actual = 0;
double omega_tilt_actual = 0;
double current_left = 0;
double current_right = 0;
double current_tilt = 0;
double current_pan = 0;
double omega_actual = 0;
double current = 0;

unsigned long lastMilli = 0;

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Quaternion base_msg;
ros::Publisher p("feedback_wheel", &base_msg);

geometry_msgs::Quaternion head_msg;
ros::Publisher p1("feedback_head", &head_msg);

void messageCb(const geometry_msgs::Quaternion& msg)
{
  omega_left_target = msg.x;  
  omega_right_target = msg.y;
  omega_pan_target = msg.z;  
  omega_tilt_target = msg.w;
  sendCmd1(omega_left_target, slave1);
  readFeedback1(slave1, &omega_left_actual, &current_left);
  delayMicroseconds(150);
  sendCmd1(omega_right_target, slave2);
  readFeedback1(slave2, &omega_right_actual, &current_right);
  delayMicroseconds(150);
  sendCmd1(omega_pan_target, slave3);
  readFeedback1(slave3, &omega_pan_actual, &current_pan);
  delayMicroseconds(150);
  sendCmd1(omega_tilt_target, slave4);
  readFeedback1(slave4, &omega_tilt_actual, &current_tilt);
}


ros::Subscriber<geometry_msgs::Quaternion> s("cmd_wheel_head_angularVel",messageCb);

void setup() 
{
  nh.getHardware()->setBaud(500000);
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  nh.advertise(p1);

  Serial1.begin (500000);  //right
  Serial2.begin (500000);  //left
  pinMode(RS485Enable1, OUTPUT);
  digitalWrite(RS485Enable1, RS485Transmit); //DE,RE=LOW, RX enabled
  pinMode(RS485Enable2, OUTPUT);
  digitalWrite(RS485Enable2, RS485Transmit); //DE,RE=LOW, RX enabled
}

void loop() 
{
  if((millis()-lastMilli) >= LOOPTIME)   
       {                                    // enter tmed loop
          lastMilli = millis();
          //left and right target
          base_msg.x = omega_left_actual;
          base_msg.y = omega_right_actual;
          base_msg.z = current_left;
          base_msg.w = current_right;
          p.publish(&base_msg);
          
          //pan and tilt target
          head_msg.x = omega_pan_actual;
          head_msg.y = omega_tilt_actual;
          head_msg.z = current_pan;
          head_msg.w = current_tilt;
          p1.publish(&head_msg);   
       }     
  nh.spinOnce();    
}

void readFeedback1(int slave, double *omega_actual, double *current)
//void readFeedback(int slave, double &omega_actual, double &current)
//double readFeedback(int slave)
{
  int actual_receive;
  //double omega_actual;
  if (Serial1.available() >= 6) 
  {  
    char rT = (char)Serial1.read(); //read actual speed from Uno
    if(rT == '{')
     {
       char commandArray[5];
       Serial1.readBytes(commandArray,5);
       byte rA = commandArray[0];
       byte rH = commandArray[1];
       byte rL = commandArray[2];
       byte rCS = commandArray[3];
       char rP = commandArray[4];
       if(rP == '}' && rA == slave)         
       {
          actual_receive = (rH << 8) + rL; 
          *omega_actual = double (actual_receive * 0.0009765923); //convert received 16 bit integer to actual speed
          *current = double (rCS*37.20238092);
          //omega_actual = double (actual_receive * 0.0009765923); //convert received 16 bit integer to actual speed
          //current = double (rCS*37.20238092);
       }  
       else
       {
          *omega_actual = 0;
          *current = 0;
       }
     }
  } 
}
void sendCmd1(double omega_target, int slave)
{
  int target_send;
  target_send = int(omega_target/0.0009765923); //convert rad/s to 16 bit integer to send
  digitalWrite(RS485Enable1, RS485Transmit); //DE,RE=HIGH, TX enabled
  char sT = '{'; //send start byte
  byte sA = slave;
  byte sH = highByte(target_send); //send high byte
  byte sL = lowByte(target_send);  //send low byte
  char sP = '}'; //send stop byte
  Serial1.write(sT); Serial1.write(sA);
  Serial1.write(sH);Serial1.write(sL); Serial1.write(sP);
  delayMicroseconds(100);
  digitalWrite(RS485Enable1, RS485Receive); //DE,RE=LOW, RX enabled
}

void sendCmd2(double omega_target, int slave)
{
  int target_send;
  target_send = int(omega_target/0.0009765923); //convert rad/s to 16 bit integer to send
  digitalWrite(RS485Enable2, RS485Transmit); //DE,RE=HIGH, TX enabled
  char sT = '{'; //send start byte
  byte sA = slave;
  byte sH = highByte(target_send); //send high byte
  byte sL = lowByte(target_send);  //send low byte
  char sP = '}'; //send stop byte
  Serial2.write(sT); Serial2.write(sA);
  Serial2.write(sH);Serial2.write(sL); Serial2.write(sP);
  delayMicroseconds(100);
  digitalWrite(RS485Enable2, RS485Receive); //DE,RE=LOW, RX enabled
} 


void readFeedback2(int slave, double *omega_actual, double *current)
//double readFeedback(int slave)
{
  int actual_receive;
  //double omega_actual;
  if (Serial2.available() >= 6) 
  {  
    char rT = (char)Serial2.read(); //read actual speed from Uno
    if(rT == '{')
     {
       char commandArray[5];
       Serial2.readBytes(commandArray,5);
       byte rA = commandArray[0];
       byte rH = commandArray[1];
       byte rL = commandArray[2];
       byte rCS = commandArray[3];
       char rP = commandArray[4];
       if(rP == '}' && rA == slave)         
       {
          actual_receive = (rH << 8) + rL; 
          *omega_actual = double (actual_receive * 0.0009765923); //convert received 16 bit integer to actual speed
          *current = double (rCS*37.20238092);
       }  
       else
       {
          *omega_actual = 0;
          *current = 0;
       }
     }
  } 
}

