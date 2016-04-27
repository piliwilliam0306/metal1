//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Range.h>
//#include "SoftwareSerial.h";


//only the following can be used for RX: 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
/*const int Tx_R = 13;
const int Rx_R = 12;

const int Tx_L = 11;
const int Rx_L = 10; 
*/
char commandArray_L[3];
byte sT_L = 0;  //send start byte
byte sH_L = 0;  //send high byte
byte sL_L = 0;  //send low byte
byte sP_L = 0;  //send stop byte

byte rT_L = 0;  //receive start byte
byte rH_L = 0;  //receive high byte
byte rL_L = 0;  //receive low byte
byte rP_L = 0;  //receive stop byte

char commandArray_R[3];
byte sT_R = 0;  //send start byte
byte sH_R = 0;  //send high byte
byte sL_R = 0;  //send low byte
byte sP_R = 0;  //send stop byte

byte rT_R = 0;  //receive start byte
byte rH_R = 0;  //receive high byte
byte rL_R = 0;  //receive low byte
byte rP_R = 0;  //receive stop byte
/*
#define trig1 22
#define echo1 23
#define trig2 24
#define echo2 25
#define trig3 26
#define echo3 27
#define trig4 28
#define echo4 29
#define trig5 30
#define echo5 31
#define trig6 32
#define echo6 33
#define trig7 34
#define echo7 35
#define trig8 36
#define echo8 37
#define trig9 38
#define echo9 39
#define trig10 40
#define echo10 41
#define trig11 42
#define echo11 43
#define trig12 44
#define echo12 45
*/
#define LOOPTIME        50
/*
SoftwareSerial mySerial_L(Rx_L, Tx_L);
SoftwareSerial mySerial_R(Rx_R, Tx_R);
*/
double omega_left_target = 0.0;
double omega_right_target = 0.0;

double omega_left_actual = 0;
double omega_right_actual = 0;
unsigned long lastMilli = 0;
long dT = 0;
int left_actual_receive = 0;
int left_target_send = 0;
int right_actual_receive = 0;
int right_target_send = 0;

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Vector3 vel_msg;
ros::Publisher p("feedback_wheel_angularVel", &vel_msg);
/*
//ultrasonic
sensor_msgs::Range range_msg1;
ros::Publisher pub_range1( "/ultrasound1", &range_msg1);

sensor_msgs::Range range_msg2;
ros::Publisher pub_range2( "/ultrasound2", &range_msg2);

sensor_msgs::Range range_msg3;
ros::Publisher pub_range3( "/ultrasound3", &range_msg3);

sensor_msgs::Range range_msg4;
ros::Publisher pub_range4( "/ultrasound4", &range_msg4);

sensor_msgs::Range range_msg5;
ros::Publisher pub_range5( "/ultrasound5", &range_msg5);

sensor_msgs::Range range_msg6;
ros::Publisher pub_range6( "/ultrasound6", &range_msg6);

sensor_msgs::Range range_msg7;
ros::Publisher pub_range7( "/ultrasound7", &range_msg7);

sensor_msgs::Range range_msg8;
ros::Publisher pub_range8( "/ultrasound8", &range_msg8);

sensor_msgs::Range range_msg9;
ros::Publisher pub_range9( "/ultrasound9", &range_msg9);

sensor_msgs::Range range_msg10;
ros::Publisher pub_range10( "/ultrasound10", &range_msg10);

sensor_msgs::Range range_msg11;
ros::Publisher pub_range11( "/ultrasound11", &range_msg11);

sensor_msgs::Range range_msg12;
ros::Publisher pub_range12( "/ultrasound12", &range_msg12);
*/
void messageCb(const geometry_msgs::Vector3& msg)
{
  omega_left_target = msg.x;  
  omega_right_target = msg.y;
}

ros::Subscriber<geometry_msgs::Vector3> s("cmd_wheel_angularVel",messageCb);

void setup() 
{
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  /*
  nh.advertise(pub_range1);
  nh.advertise(pub_range2);
  nh.advertise(pub_range3);
  nh.advertise(pub_range4);
  nh.advertise(pub_range5);
  nh.advertise(pub_range6);
  nh.advertise(pub_range7);
  nh.advertise(pub_range8);
  nh.advertise(pub_range9);
  nh.advertise(pub_range10);
  nh.advertise(pub_range11);
  nh.advertise(pub_range12);

  range_msg1.header.frame_id =  "/ultrasound1";
  range_msg2.header.frame_id =  "/ultrasound2";
  range_msg3.header.frame_id =  "/ultrasound3";
  range_msg4.header.frame_id =  "/ultrasound4";
  range_msg5.header.frame_id =  "/ultrasound5";
  range_msg6.header.frame_id =  "/ultrasound6";
  range_msg7.header.frame_id =  "/ultrasound7";
  range_msg8.header.frame_id =  "/ultrasound8";
  range_msg9.header.frame_id =  "/ultrasound9";
  range_msg10.header.frame_id =  "/ultrasound10";
  range_msg11.header.frame_id =  "/ultrasound11";
  range_msg12.header.frame_id =  "/ultrasound12";
  */
  /*
  pinMode(Rx_L, INPUT); pinMode(Tx_L, OUTPUT);
  pinMode(Rx_R, INPUT); pinMode(Tx_R, OUTPUT);
  mySerial_L.begin (19200);
  mySerial_R.begin (19200);
  */
  //The Arduino Mega has three additional serial ports: 
    //Serial1 on pins 19 (RX) and 18 (TX), 
    //Serial2 on pins 17 (RX) and 16 (TX), 
    //Serial on pins 15 (RX) and 14 (TX). 
  //Serial.begin (19200);
  Serial2.begin (57600);  //left
  Serial1.begin (57600);  //right
}

void loop() 
{
  readFeadback_angularVel_L();
  readFeadback_angularVel_R();   
  if((millis()-lastMilli) >= LOOPTIME)   
       {                                    // enter tmed loop
          dT = millis()-lastMilli;
          lastMilli = millis();

          //readFeadback_angularVel_L();
          //delay(50);
          //readFeadback_angularVel_R();  
          sendCmd_wheel_angularVel_L();
          sendCmd_wheel_angularVel_R();

          vel_msg.x=omega_left_actual;
          vel_msg.y=omega_right_actual;
          p.publish(&vel_msg);
          /*
          range_msg1.range = 0;
          pub_range1.publish(&range_msg1);
          
          range_msg2.range = 0;
          pub_range2.publish(&range_msg2);

          range_msg3.range = 0;
          pub_range3.publish(&range_msg3);

          range_msg4.range = 0;
          pub_range4.publish(&range_msg4);
          
          range_msg5.range = 0;
          pub_range5.publish(&range_msg5);

          range_msg6.range = 0;
          pub_range6.publish(&range_msg6);

          range_msg7.range = 0;
          pub_range7.publish(&range_msg7);
          
          range_msg8.range = 0;
          pub_range8.publish(&range_msg8);

          range_msg9.range = 0;
          pub_range9.publish(&range_msg9);

          range_msg10.range = 0;
          pub_range10.publish(&range_msg10);

          range_msg11.range = 0;
          pub_range11.publish(&range_msg11);

          range_msg12.range = 0;
          pub_range12.publish(&range_msg12);
          */
          nh.spinOnce();
          //printMotorInfo();
       }     
      
}

void readFeadback_angularVel_L()
{
  //if (mySerial_L.available() > 4) 
  if (Serial2.available() >= 4) 
  {
    //char rT_L = (char)mySerial_L.read(); //read actual speed from Uno
    char rT_L = (char)Serial2.read(); //read actual speed from Uno
    if(rT_L == '{')
      {
        char commandArray_L[3];
        //mySerial_L.readBytes(commandArray_L,3);
        Serial2.readBytes(commandArray_L,3);
        byte rH_L = commandArray_L[0];
        byte rL_L = commandArray_L[1];
        char rP_L = commandArray_L[2];
        if(rP_L == '}')         
          {
            left_actual_receive = (rH_L << 8) + rL_L; 
            omega_left_actual = double (left_actual_receive * 0.00031434064); //convert received 16 bit integer to actual speed
          }
      }   
  }
}

void readFeadback_angularVel_R()
{
  //if (mySerial_R.available() > 4) 
  if (Serial1.available() >= 4) 
  {  
    //char rT_R = (char)mySerial_R.read(); //read actual speed from Uno
    char rT_R = (char)Serial1.read(); //read actual speed from Uno
    if(rT_R == '{')
     {
       char commandArray_R[3];
       //mySerial_R.readBytes(commandArray_R,3);
       Serial1.readBytes(commandArray_R,3);
       byte rH_R = commandArray_R[0];
       byte rL_R = commandArray_R[1];
       char rP_R = commandArray_R[2];
       if(rP_R == '}')         
       {
        right_actual_receive = (rH_R << 8) + rL_R; 
        omega_right_actual = double (right_actual_receive * 0.00031434064); //convert received 16 bit integer to actual speed
       }  
     }
  }   
}

void sendCmd_wheel_angularVel_L()
{
  left_target_send = int(omega_left_target/0.00031434064); //convert rad/s to 16 bit integer to send
  char sT_L = '{'; //send start byte
  byte sH_L = highByte(left_target_send); //send high byte
  byte sL_L = lowByte(left_target_send);  //send low byte
  char sP_L = '}'; //send stop byte
  //mySerial_L.write(sT_L); mySerial_L.write(sH_L); mySerial_L.write(sL_L); mySerial_L.write(sP_L);
  Serial2.write(sT_L); Serial2.write(sH_L); Serial2.write(sL_L); Serial2.write(sP_L);
}


void sendCmd_wheel_angularVel_R()
{
  right_target_send = int(omega_right_target/0.00031434064); //convert rad/s to 16 bit integer to send
  char sT_R = '{'; //send start byte
  byte sH_R = highByte(right_target_send); //send high byte
  byte sL_R = lowByte(right_target_send);  //send low byte
  char sP_R = '}'; //send stop byte
  //mySerial_R.write(sT_R); mySerial_R.write(sH_R); mySerial_R.write(sL_R); mySerial_R.write(sP_R);
  Serial1.write(sT_R); Serial1.write(sH_R); Serial1.write(sL_R); Serial1.write(sP_R);
}


void printMotorInfo()  
{                                                                      
   Serial.print("  LEFT target rad/s:");                  Serial.print(omega_left_target);
   Serial.print("  LEFT actual rad/s     :");                  Serial.print(omega_left_actual);

   Serial.print("  RIGHT target rad/s:");                  Serial.print(omega_right_target);
   Serial.print("  RIGHT actual rad/s:");                  Serial.print(omega_right_actual);
   Serial.println(); 
   Serial.println(); 

}




