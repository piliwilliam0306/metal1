
#include <ros.h>
//#include <geometry_msgs/Vector3.h>
#include <andbot/JointCmd.h>
#include <andbot/Calibrate.h>
#include <andbot/DriverState.h>

#define LOOPTIME 100

#define RS485Transmit    HIGH
#define RS485Receive     LOW 

#define slave0 0
#define slave1 1
#define slave2 2
#define slave3 3

unsigned long lastMilli = 0;                    // loop timing 
long dT = 0;
unsigned long cc = 0;

//double rad_tick=(double)62.83/32767; //10rev 2^15 //6.28 * 10
//double degree_tick=(double)3600/32767; //

double rad_tick=(double)6.283/32767; //10rev 2^15 //6.28 * 10
double degree_tick=(double)360/32767; //

bool driver_mode1;
bool device_available = false;

byte current = 0;

byte Kp = 0;
byte Ki = 0;
byte Kd = 0;
byte Current_Limit = 0;

ros::NodeHandle nh;
using andbot::Calibrate;

bool set_; 

void messageCb1(const andbot::JointCmd& cmd_msg)
{
  double theta_target0 = 0.0;
  double theta_target1 = 0.0;
  double theta_target2 = 0.0;
  double theta_target3 = 0.0;
  andbot::Cmd joint0 = cmd_msg.joint0;
  andbot::Cmd joint1 = cmd_msg.joint1;
  andbot::Cmd joint2 = cmd_msg.joint2;
  andbot::Cmd joint3 = cmd_msg.joint3;
  theta_target0 = joint0.angle;  
  sendCmd_L(theta_target0, slave0);
  theta_target1 = joint1.angle;  
  sendCmd_L(theta_target1, slave1);
  theta_target2 = joint2.angle;  
  sendCmd_L(theta_target2, slave2);
  theta_target3 = joint3.angle;  
  sendCmd_L(theta_target3, slave3);
}

void messageCb2(const andbot::JointCmd& cmd_msg)
{
  double theta_target0 = 0.0;
  double theta_target1 = 0.0;
  double theta_target2 = 0.0;
  double theta_target3 = 0.0;
  andbot::Cmd joint0 = cmd_msg.joint0;
  andbot::Cmd joint1 = cmd_msg.joint1;
  andbot::Cmd joint2 = cmd_msg.joint2;
  andbot::Cmd joint3 = cmd_msg.joint3;
  theta_target0 = joint0.angle;  
  sendCmd_R(theta_target0, slave0);
  theta_target1 = joint1.angle;  
  sendCmd_R(theta_target1, slave1);
  theta_target2 = joint2.angle;  
  sendCmd_R(theta_target2, slave2);
  theta_target3 = joint3.angle;  
  sendCmd_R(theta_target3, slave3);
}

ros::Subscriber<andbot::JointCmd> s1("cmd_L",messageCb1);
ros::Subscriber<andbot::JointCmd> s2("cmd_R",messageCb2);

void callback(const Calibrate::Request & req, Calibrate::Response & res){
    Current_Limit = req.currentlim; Kp = req.kp; Ki = req.ki; Kd = req.kd; 
    CalCmd_L(req.id);
    res.id = req.id;  res.kp = req.kp;  res.ki = req.ki;  res.kd = req.kd;  res.currentlim = req.currentlim;
}

void callback1(const Calibrate::Request & req, Calibrate::Response & res){
    Current_Limit = req.currentlim; Kp = req.kp; Ki = req.ki; Kd = req.kd; 
    CalCmd_R(req.id);
    res.id = req.id;  res.kp = req.kp;  res.ki = req.ki;  res.kd = req.kd;  res.currentlim = req.currentlim;
}

ros::ServiceServer<Calibrate::Request, Calibrate::Response> server("joint_calibration_L",&callback);
ros::ServiceServer<Calibrate::Request, Calibrate::Response> server1("joint_calibration_R",&callback1);

void setup() { 
  TCCR0B = TCCR0B & B11111000 | B00000010;
  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.subscribe(s1); nh.subscribe(s2); 
  nh.advertiseService(server);
  nh.advertiseService(server1);
  pinMode(8, OUTPUT);
  //digitalWrite(8, RS485Receive); //DE,RE=LOW, RX enabled
  digitalWrite(8, RS485Transmit); //DE,RE=HIGH, TX enabled
  Serial1.begin (2000000);  //left arm
  Serial2.begin (2000000);  //right arm
} 

void loop() 
{       
  /*if((millis()-lastMilli) >= LOOPTIME)   
    {                                    // enter tmed loop
      dT = millis()-lastMilli;
      lastMilli = millis();

    }*/
  nh.spinOnce();
}

void CalCmd_L(byte slave)
{
  byte buf[7];
  buf[0] = '|';
  buf[1] = slave;
  buf[2] = Kp; buf[3] = Ki; buf[4] = Kd; //send PID
  buf[5] = Current_Limit; //current limit
  buf[6] ='|';
  Serial1.write(buf, sizeof(buf));
}

void CalCmd_R(byte slave)
{
  byte buf[7];
  buf[0] = '|';
  buf[1] = slave;
  buf[2] = Kp; buf[3] = Ki; buf[4] = Kd; //send PID
  buf[5] = Current_Limit; //current limit
  buf[6] ='|';
  Serial2.write(buf, sizeof(buf));
}

void sendCmd_L(double theta_target,byte slave)
{
  int target_send;
  target_send = int(theta_target/degree_tick); //convert rad/s to 16 bit integer to send  100*6.28/2^16
  byte buf[5];
  buf[0] = '{';
  buf[1] = slave;
  buf[2] = highByte(target_send); buf[3] = lowByte(target_send);
  buf[4] ='}';
  Serial1.write(buf, sizeof(buf));
}

void sendCmd_R(double theta_target,byte slave)
{
  int target_send;
  target_send = int(theta_target/rad_tick); //convert rad/s to 16 bit integer to send  100*6.28/2^16
  byte buf[5];
  buf[0] = '{';
  buf[1] = slave;
  buf[2] = highByte(target_send); buf[3] = lowByte(target_send);
  buf[4] ='}';
  Serial2.write(buf, sizeof(buf));
}
/*
void printMotorInfo()  
{                                                                      
   Serial.print(" target1:");                  Serial.print(theta_target1);
   //Serial.print(" actual1:");                  Serial.print(omega_actual1);
   //Serial.print(" actual:");                  Serial.print(omega_actual);
   Serial.println();
   Serial.print(" target2:");                  Serial.print(theta_target2);
   Serial.print(" actual2:");                  Serial.print(omega_actual2);
   Serial.println();
}
*/
