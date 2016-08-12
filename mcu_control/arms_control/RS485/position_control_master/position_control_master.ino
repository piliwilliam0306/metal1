#include <ros.h>
#include <andbot/JointCmd.h>
#include <andbot/JointOffset.h>
//#include <andbot/Calibrate.h>
#include <andbot/Cmd.h>
#include <andbot/Offset.h>
#include <andbot/DriverState.h>
#include <SoftwareSerial.h>

#define RS485Transmit    HIGH
#define RS485Receive     LOW 
#define RS485Mode1 8  //pin8 for changing 
#define RS485Mode2 9
//#define SLAVE   4 //up to 8 slaves
#define LOOPTIME 20

#define slave0 0
#define slave1 1
#define slave2 2
#define slave3 3

double rad_tick = 0.0002; //6.283/32767
double  degree_tick = 0.011; //360/32767
double  rad_to_deg_byte = 0.00175; //360/6.28/32767
double  rad_to_deg = 57.325; //360/6.28

int offset_L0 = 0, offset_L1 = 0, offset_L2 = 0, offset_L3 = 0;
int offset_R0 = 0, offset_R1 = 0, offset_R2 = 0, offset_R3 = 0;

double increment_angle_0=0.2, increment_angle_1=0.2;

double gearRatio_Axis_0 = 5;
double gearRatio_Axis_1 = 3;
double gearRatio_Axis_2 = 2.5;
double gearRatio_Axis_3 = 3;

//double gearRatio_Axis_0 = 0;
//double gearRatio_Axis_1 = 0;

bool ARM_RIGHT_A0_A1_EN =0;
bool ARM_RIGHT_A2_A3_EN =0;
bool ARM_LEFT_A0_A1_EN = 0;
bool ARM_LEFT_A2_A3_EN = 0;

double cmd_Motor_L0 = 0, cmd_Motor_L1 = 0;
double cmd_Motor_L2 = 0, cmd_Motor_L3 = 0;
double cmd_Motor_R0 = 0, cmd_Motor_R1 = 0;
double cmd_Motor_R2 = 0, cmd_Motor_R3 = 0;

double right_angle_Axis_0 = 0, right_angle_Axis_1 = 0;
double right_angle_Axis_2 = 0, right_angle_Axis_3 = 0;
double left_angle_Axis_0 = 0, left_angle_Axis_1 = 0;
double left_angle_Axis_2 = 0, left_angle_Axis_3 = 0;

byte current = 0;
byte ID = 0;
byte Kp = 0;
byte Ki = 0;
byte Kd = 0;
byte Current_Limit = 0;

double position_target = 0;
double MaxAngle = 360;

unsigned long lastMilli = 0;
long dT = 0;

ros::NodeHandle nh;
//using andbot::Calibrate;
bool set_; 

void messageCallBackHomeArm_L(const andbot::JointOffset& offset_msg)
{
  andbot::Offset joint0 = offset_msg.joint0;
  andbot::Offset joint1 = offset_msg.joint1;
  andbot::Offset joint2 = offset_msg.joint2;
  andbot::Offset joint3 = offset_msg.joint3;
  //need to convert to deg/s since unit is rad/s from ROS
  
  offset_L0 = (int)(joint0.offset*gearRatio_Axis_0*(double)360/6.28)/0.01;
  offset_L1 = (int)(joint1.offset*gearRatio_Axis_1*(double)360/6.28)/0.01;
  offset_L2 = (int)(joint2.offset*gearRatio_Axis_2*(double)360/6.28)/0.01;
  offset_L3 = (int)(joint3.offset*gearRatio_Axis_3*(double)360/6.28)/0.01;
  
  sendCmd_L(offset_L0, cmd_Motor_L0, slave0);
  sendCmd_L(offset_L1, cmd_Motor_L1, slave1);
  sendCmd_L(offset_L2, cmd_Motor_L2, slave2); 
  sendCmd_L(offset_L3, cmd_Motor_L3, slave3);
}

void messageCallBackHomeArm_R(const andbot::JointOffset& offset_msg)
{
  andbot::Offset joint0 = offset_msg.joint0;
  andbot::Offset joint1 = offset_msg.joint1;
  andbot::Offset joint2 = offset_msg.joint2;
  andbot::Offset joint3 = offset_msg.joint3;
  //need to convert to deg/s since unit is rad/s from ROS
  offset_R0 = (int)(joint0.offset*gearRatio_Axis_0*(double)360/6.28)/0.01;
  offset_R1 = (int)(joint1.offset*gearRatio_Axis_1*(double)360/6.28)/0.01;
  offset_R2 = (int)(joint2.offset*gearRatio_Axis_2*(double)360/6.28)/0.01;
  offset_R3 = (int)(joint3.offset*gearRatio_Axis_3*(double)360/6.28)/0.01;
  
  sendCmd_R(offset_R0, cmd_Motor_R0, slave0);
  sendCmd_R(offset_R1, cmd_Motor_R1, slave1);
  sendCmd_R(offset_R2, cmd_Motor_R2, slave2); 
  sendCmd_R(offset_R3, cmd_Motor_R3, slave3);
}

//double angle_Axis_0 = 0, angle_Axis_1 = 0;

void messageCallBackArm_L(const andbot::JointCmd& cmd_msg)
{
  andbot::Cmd joint0 = cmd_msg.joint0;
  andbot::Cmd joint1 = cmd_msg.joint1;
  andbot::Cmd joint2 = cmd_msg.joint2;
  andbot::Cmd joint3 = cmd_msg.joint3;

  left_angle_Axis_0=(int)(joint0.angle*(double)360/6.28);  
  left_angle_Axis_1=(int)(joint1.angle*(double)360/6.28);  
  left_angle_Axis_2=(int)(joint2.angle*(double)360/6.28); 
  left_angle_Axis_3=(int)(joint3.angle*(double)360/6.28); 
}

void messageCallBackArm_R(const andbot::JointCmd& cmd_msg)
{
  andbot::Cmd joint0 = cmd_msg.joint0;
  andbot::Cmd joint1 = cmd_msg.joint1;
  andbot::Cmd joint2 = cmd_msg.joint2;
  andbot::Cmd joint3 = cmd_msg.joint3;

  right_angle_Axis_0=(int)(joint0.angle*(double)360/6.28);  
  right_angle_Axis_1=(int)(joint1.angle*(double)360/6.28);  
  right_angle_Axis_2=(int)(joint2.angle*(double)360/6.28);
  right_angle_Axis_3=(int)(joint3.angle*(double)360/6.28);
}

ros::Subscriber<andbot::JointOffset> s1("andbot/joint/L/cmd/offset_v", messageCallBackHomeArm_L);
ros::Subscriber<andbot::JointOffset> s2("andbot/joint/R/cmd/offset_v", messageCallBackHomeArm_R);
ros::Subscriber<andbot::JointCmd> s3("andbot/joint/L/cmd/position", messageCallBackArm_L);
ros::Subscriber<andbot::JointCmd> s4("andbot/joint/R/cmd/position", messageCallBackArm_R);
/*
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
*/
void setup() {
  pinMode(RS485Mode1, OUTPUT); digitalWrite(RS485Mode1, RS485Transmit); //DE,RE=LOW, RX enabled
  pinMode(RS485Mode2, OUTPUT); digitalWrite(RS485Mode2, RS485Transmit);
  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.subscribe(s1); nh.subscribe(s2); nh.subscribe(s3); nh.subscribe(s4);
  //nh.advertiseService(server);
  //nh.advertiseService(server1);
  //Serial.begin(57600);     // Console 
  Serial1.begin(1000000);  // TO MAX485
  Serial2.begin(1000000);  // TO Logic
}


void loop() 
{
  if((millis()-lastMilli) >= LOOPTIME)  
  {                        
    lastMilli = millis();
    LEFT_A0_A1();
    LEFT_A2_A3();    
    RIGHT_A0_A1();
    RIGHT_A2_A3();

    sendCmd_L(offset_L0, cmd_Motor_L0, slave0);
    sendCmd_L(offset_L1, cmd_Motor_L1, slave1);
    sendCmd_L(offset_L2, cmd_Motor_L2, slave2); 
    sendCmd_L(offset_L3, cmd_Motor_L3, slave3);
    sendCmd_R(offset_R0, cmd_Motor_R0, slave0);
    sendCmd_R(offset_R1, cmd_Motor_R1, slave1);
    sendCmd_R(offset_R2, cmd_Motor_R2, slave2); 
    sendCmd_R(offset_R3, cmd_Motor_R3, slave3);
    
    //show();
  }
  nh.spinOnce();
}

void show()
  {
    Serial.print("L0: ");Serial.print(cmd_Motor_L0);  Serial.print(" ");
    Serial.print("L1: ");Serial.print(cmd_Motor_L1);  Serial.print(" ");
    Serial.print("L2: ");Serial.print(cmd_Motor_L2);  Serial.print(" ");
    Serial.print("L3: ");Serial.print(cmd_Motor_L3);  Serial.print(" ");
    Serial.print("R0: ");Serial.print(cmd_Motor_R0);  Serial.print(" ");
    Serial.print("R1: ");Serial.print(cmd_Motor_R1);  Serial.print(" ");
    Serial.print("R2: ");Serial.print(cmd_Motor_R2);  Serial.print(" ");
    Serial.print("R3: ");Serial.print(cmd_Motor_R3);  Serial.print(" ");
    Serial.println(" ");    
    }

