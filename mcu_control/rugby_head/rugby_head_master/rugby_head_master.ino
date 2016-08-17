#include <ros.h>
#include <andbot/HeadCmd.h>
#include <andbot/HeadOffset.h>

#include <SoftwareSerial.h>

#define RS485Transmit    HIGH
#define RS485Receive     LOW 
#define RS485Mode1 8  //pin8 for changing 

#define LOOPTIME 20

#define slave0 0
#define slave1 1

double rad_tick = 0.0002; //6.283/32767
double  degree_tick = 0.011; //360/32767
double  rad_to_deg_byte = 0.00175; //360/6.28/32767
double  rad_to_deg = 57.325; //360/6.28

int offset_P = 0, offset_T = 0;

double increment_angle_0=0.2, increment_angle_1=0.2;

double gearRatio_Axis_0 = 1;
double gearRatio_Axis_1 = 3;

bool ARM_RIGHT_A0_A1_EN =0;
bool ARM_RIGHT_A2_A3_EN =0;
bool ARM_LEFT_A0_A1_EN = 1;
bool ARM_LEFT_A2_A3_EN = 0;

double cmd_Motor_P = 0, cmd_Motor_T = 0;


double position_target = 0;
double MaxAngle = 360;

unsigned long lastMilli = 0;
long dT = 0;

ros::NodeHandle nh;
bool set_; 

double pan_angle_Axis = 0, tilt_angle_Axis = 0;

void messageCallBackHome(const andbot::HeadOffset& offset_msg)
{
  //need to convert to deg/s since unit is rad/s from ROS
  
  offset_P = (int)(offset_msg.speed1*gearRatio_Axis_0*(double)360/6.28)/0.01;
  offset_T = (int)(offset_msg.speed2*gearRatio_Axis_1*(double)360/6.28)/0.01;
  
  sendCmd(offset_P, cmd_Motor_P, slave0);
  sendCmd(offset_T, cmd_Motor_T, slave1);
}

void messageCallBackHead(const andbot::HeadCmd& cmd_msg)
{
  pan_angle_Axis=(int)(cmd_msg.angle1*(double)360/6.28);  
  tilt_angle_Axis=(int)(cmd_msg.angle2*(double)360/6.28);  
}

ros::Subscriber<andbot::HeadOffset> s1("rugby/Head/cmd/offset_v", messageCallBackHome);
ros::Subscriber<andbot::HeadCmd> s2("rugby/Head/cmd/position", messageCallBackHead);

void setup() {
  pinMode(RS485Mode1, OUTPUT); digitalWrite(RS485Mode1, RS485Transmit); //DE,RE=LOW, RX enabled

  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.subscribe(s1); nh.subscribe(s2);

  //Serial.begin(57600);     // Console 
  Serial3.begin(1000000);  // TO MAX485
}


void loop() 
{
  if((millis()-lastMilli) >= LOOPTIME)  
  {                        
    lastMilli = millis();
    LEFT_A0_A1();

    sendCmd(offset_P, cmd_Motor_P, slave0);
    sendCmd(offset_T, cmd_Motor_T, slave1);
    //show();
  }
  nh.spinOnce();
}
/*
void show()
  {
    Serial.print("Pan: ");Serial.print(cmd_Motor_P);  Serial.print(" ");
    Serial.print("Tilt: ");Serial.print(cmd_Motor_T);  Serial.print(" ");
    Serial.println(" ");    
    }
*/
