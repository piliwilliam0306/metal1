//This code uses direct port manipulation which only works on Arduino Mega 2560

#include <ros.h>
#include <WheelCmd.h>
#include <WheelFb.h>
#include <avr/io.h>
#include <DriverState.h>

#define MaxSpeed 19

#define RS485Transmit    HIGH
#define RS485Receive     LOW

#define LOOPTIME        80

double omega_left_target = 0.0;
double omega_right_target = 0.0;
double omega_left_actual = 0;
double omega_right_actual = 0;
unsigned long lastMilli = 0;
unsigned long lastBool = 0;

long dT = 0;

#define TimeOut 5000//TimeOut = Max.Distance(cm) * 58

unsigned int current_left = 0;
unsigned int current_right = 0;
bool driver_mode;

ros::NodeHandle nh;
using rugby::DriverState;
bool set_;

rugby::WheelFb wheel_msg;
rugby::WheelCmd debug;

ros::Publisher p("feedback_wheel_angularVel", &wheel_msg);
ros::Publisher d("debug", &debug);


void sendCmd_wheel_angularVel_L()
{
  digitalWrite(8, RS485Transmit);
  int left_target_send = int(omega_left_target / (double(MaxSpeed) / 32767)); //convert rad/s to 16 bit integer to send
  byte buf[6];
  buf[0] = '{'; //send start byte
  buf[1] = 2; //send address  LEFT_WHEEL 2
  buf[2] = highByte(left_target_send); //send high byte
  buf[3] = lowByte(left_target_send);  //send low byte
  if (driver_mode == true)  buf[4] = '}'; //send stop byte motor on
  if (driver_mode == false)  buf[4] = '|'; //send stop byte motor off
  buf[5] = 8;
  Serial1.write(buf, sizeof(buf));
  delayMicroseconds(25);
  digitalWrite(8, RS485Receive);
  delay(6);
  readFeadback_angularVel_L();
}

void sendCmd_wheel_angularVel_R()
{
  digitalWrite(8, RS485Transmit);
  int right_target_send = int(omega_right_target / (double(MaxSpeed) / 32767)); //convert rad/s to 16 bit integer to send
  byte buf[6];
  buf[0] = '{'; //send start byte
  buf[1] = 1;  //send address RIGHT_WHEEL 1
  buf[2] = highByte(right_target_send); //send high byte
  buf[3] = lowByte(right_target_send);  //send low byte
  buf[4] = '}'; //send stop byte motor on
  buf[5] = 8;
  Serial1.write(buf, sizeof(buf));
  delayMicroseconds(25);
  digitalWrite(8, RS485Receive);
  delay(6);
  readFeadback_angularVel_R();
}

//callback
void messageCb(const rugby::WheelCmd& msg)
{
  omega_left_target = msg.speed1;
  omega_right_target = msg.speed2;
  driver_mode = msg.driverstate;

  debug.speed1 = msg.speed1;
  debug.speed2 = msg.speed2;
  debug.driverstate = msg.driverstate;
  //d.publish(&debug);
}

ros::Subscriber<rugby::WheelCmd> s("cmd_wheel_angularVel", messageCb);

void setup()
{
  pinMode(8, OUTPUT);

  //TCCR0B = TCCR0B & B11111000 | B00000010;
  //set baud rate for rosserial
  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  nh.advertise(d);

  Serial1.begin(1000000);
  Serial2.begin(1000000);
}

void loop()
{
  if ((millis() - lastMilli) >= LOOPTIME)
  { // enter tmed loop
    dT = millis() - lastMilli;
    lastMilli = millis();

    sendCmd_wheel_angularVel_L();
    delay(1);
    sendCmd_wheel_angularVel_R();
    delay(1);

    wheel_msg.speed1 = omega_left_actual;
    wheel_msg.speed2 = omega_right_actual;
    wheel_msg.current1 = current_left;
    wheel_msg.current2 = current_right;

    p.publish(&wheel_msg);
  }
  nh.spinOnce();
}


void readFeadback_angularVel_L()
{
  digitalWrite(8, RS485Receive);
  int actual_receive;
  if (Serial1.available() >= 6)
  {
    char rT_L = (char)Serial1.read(); //read actual speed from Uno
    if (rT_L == '{')
    {
      char commandArray_L[6];
      Serial1.readBytes(commandArray_L, 6);
      byte rA = commandArray_L[0];
      byte rH_L = commandArray_L[1];
      byte rL_L = commandArray_L[2];
      byte rCS_L = commandArray_L[3];
      char rP_L = commandArray_L[4];
      byte rG = commandArray_L[5];
      if (rP_L == '}' && rA == 1)
      {
        actual_receive = (rH_L << 8) + rL_L;
        omega_left_actual = double (actual_receive * (double(MaxSpeed) / 32767)); //convert received 16 bit integer to actual speed
        //max current is 20400mA, 255 * 80 = 20400mA
        current_left = (rCS_L * 80);
      }
    }
  }
}

void readFeadback_angularVel_R()
{
  digitalWrite(8, RS485Receive);
  int actual_receive;
  if (Serial1.available() >= 6)
  {
    char rT_R = (char)Serial1.read(); //read actual speed from Uno
    if (rT_R == '{')
    {
      char commandArray_R[6];
      Serial1.readBytes(commandArray_R, 6);
      byte rA = commandArray_R[0];
      byte rH_R = commandArray_R[1];
      byte rL_R = commandArray_R[2];
      byte rCS_R = commandArray_R[3];
      char rP_R = commandArray_R[4];
      byte rG = commandArray_R[5];
      if (rP_R == '}' && rA == 2)
      {
        actual_receive = (rH_R << 8) + rL_R;
        omega_right_actual = double (actual_receive * (double(MaxSpeed) / 32767)); //convert received 16 bit integer to actual speed
        current_right = rCS_R * 80;
      }
    }
  }
}

