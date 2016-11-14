/*
 * =====================================================================================
 *
 *       Filename:  mega_base_MIMO_andbot1dot2.ino
 *
 *    Description:  The program is for wheel hub motor control (both left & right).
 *                  Vq max/min is 1000/-1000
 *                  Id max/min is 20/-20 A
 *
 *                  [HW Arduino Mega 2560]
 *                  Serial port (Default serial for Connect ROSSerial )
 *                  Serial1 port (connect to Motor control board Right wheel)
 *                  Serial2 port (connect to Motor control board Left wheel)
 *                  Serial3 port (connect to BT (Test only))
 *
 *        Version:  20161019
 *        Created:
 *       Revision:  none
 *       Compiler:
 *
 *         Author:
 *        Company:  AR
 *
 * =====================================================================================
 */

#include <ArduinoHardware.h>
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Range.h>
#include <WheelCmd.h>
#include <WheelFb.h>
#include <DriverState.h>
#include <Metro.h>

union Data_Setting {
  struct _ByteSet {
    byte L;
    byte H;
  } Byte;
  int Data;
};

char val;
char commandArray_L[3];

// send/receive data through serial
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

#define LOOPTIME 40//100
unsigned long lastMilli = 0;
long dT = 0;

//parameters
int CPR = 90;                                                                     //encoder count per revolution
int gear_ratio = 1;
int actual_send = 0;
const double MAX_AngularSpeed = 47.1238898 ;//  450 / 60 * 2 * PI => DD motor nominal rotation speed: 450 rpm

int target_receive = 0;

#define Volt_MAX 12
#define Volt_MIN -12
#define Vq_MAX 1000
#define Vq_MIN -1000
const double Vq_formatRatio=1499/12; //convert voltage to Vq voltage format ( 12 (max voltage) : 1499 (Vq format max) )

double volt_left_target = 0.0;
double volt_right_target = 0.0;
double omega_left_actual = 0;
double omega_right_actual = 0;
bool driverEn;

unsigned long lastMilli = 0;
long dT = 0;
int left_actual_receive = 0;
int left_target_send = 0;
int right_actual_receive = 0;
int right_target_send = 0;

union Data_Setting MotorData_left[3];
byte getData_left[8];
byte sendData_left[7] = {123, 0, 0, 0, 0, 0, 125};
byte sendDataStop_left[7] = {123, 0, 0, 0, 0, 85, 125};

union Data_Setting MotorData_right[3];
byte getData_right[8];
byte sendData_right[7] = {123, 0, 0, 0, 0, 0, 125};
byte sendDataStop_right[7] = {123, 0, 0, 0, 0, 85, 125};

unsigned long PastTime = 0;
int Vq_left = 0, Vd_left = 0, checksum_left = 0;
int Vq_right = 0, Vd_right = 0, checksum_right = 0;

//encoder pin assignment
//left wheel
#define encodePinA_left 2
#define encodePinB_left 3
#define encodePinC_left 21
volatile long Encoderpos_left = 0;
long EncoderposPre_left = 0;
volatile int lastEncoded_left = 0;

int pinAState_left = 0;
int pinAStateOld_left = 0;
int pinBState_left = 0;
int pinBStateOld_left = 0;
int pinCState_left = 0;
int pinCStateOld_left = 0;
int EncodeDiff_left = 0;
int EncodeDiffPre_left = 0;

//right wheel
#define encodePinA_right
#define encodePinB_right
#define encodePinC_right
volatile long Encoderpos_right = 0;
long EncoderposPre_right = 0;
volatile int lastEncoded_right = 0;

int pinAState_right = 0;
int pinAStateOld_right = 0;
int pinBState_right = 0;
int pinBStateOld_right = 0;
int pinCState_right = 0;
int pinCStateOld_right = 0;
int EncodeDiff_right = 0;
int EncodeDiffPre_right = 0;

double vol_target_left = 0;
double omega_actual_left = 0;
double vol_target_right = 0;
double omega_actual_left = 0;

void sendCmd_wheel_volt_L()
{
    if(volt_left_target>Volt_MAX)           volt_left_target=Volt_MAX;
    else if(volt_left_target<Volt_MIN)      volt_left_target=Volt_MIN;

    left_target_send = int(volt_left_target / (double(Volt_MAX)/double(32767)));   //convert received 16 bit integer to actual voltage => Vq_MAX/32767

    //transmit command to the lower level mega board.
    char sT_L = '{'; //send start byte
    byte sH_L = highByte(left_target_send);
    byte sL_L = lowByte(left_target_send);
    char sP_L = '}';
    Serial2.write(sT_L); Serial2.write(sH_L); Serial2.write(sL_L); Serial2.write(sP_L);
}


void sendCmd_wheel_volt_R()
{
    if(volt_right_target>Volt_MAX)          volt_right_target=Volt_MAX;
    else if(volt_right_target<Volt_MIN)     volt_right_target=Volt_MIN;

    right_target_send = int(volt_right_target / (double(Volt_MAX)/double(32767)));   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904

    //transmit command to the lower level mega board.
    char sT_R = '{';
    byte sH_R = highByte(right_target_send);
    byte sL_R = lowByte(right_target_send);
    char sP_R = '}';
    Serial1.write(sT_R); Serial1.write(sH_R); Serial1.write(sL_R); Serial1.write(sP_R);
}

void DriverState_service_callback(const andbot1dot2::DriverStateRequest& req, andbot1dot2::DriverStateResponse& res)
{
    driverEn = req.driverstate;
    if (driverEn == true)
    {
      res.driverstate = true;
      Serial1.write("m", 1);
      Serial2.write("m", 1);
    }
    else
    {
      res.driverstate = false;
      Serial1.write("k", 1);
      Serial2.write("k", 1);
    }
    Serial.print("From Client");
    Serial.println(req.driverstate,DEC);
    Serial.print("Server says");
    Serial.print(res.driverstate,DEC);
}

void cmd_wheel_voltCb(const andbot1dot2::WheelCmd& msg)
{
    volt_left_target = msg.speed1;
    volt_right_target = msg.speed2;
    sendCmd_wheel_volt_L();
    sendCmd_wheel_volt_R();
}

void readFeadback_angularVel_L()
{
    if (Serial2.available() >= 4)
    {
        char rT_L = (char)Serial2.read();
        if (rT_L == '{')
        {
            char commandArray_L[3];
            Serial2.readBytes(commandArray_L,3);
            byte rH_L = commandArray_L[0];
            byte rL_L = commandArray_L[1];
            char rP_L = commandArray_L[2];
            if (rP_L == '}')
            {
                left_actual_receive =(rH_L << 8) + rL_L;
                omega_left_actual = double (left_actual_receive * (MAX_AngularSpeed/double(32767)));   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
            }
        }
    }
}

void readFeadback_angularVel_R()
{
    if (Serial1.available() >= 4)
    {
        char rT_R = (char)Serial1.read();
        if (rT_R == '{')
        {
            char commandArray_R[3];
            Serial1.readBytes(commandArray_R, 3);
            byte rH_R = commandArray_R[0];
            byte rL_R = commandArray_R[1];
            char rP_R = commandArray_R[2];
            if (rP_R == '}')
            {
                right_actual_receive = (rH_R << 8) + rL_R;
                omega_right_actual = double (right_actual_receive * (MAX_AngularSpeed/double(32767)));   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
            }
        }
    }
}

void doEncoder_left() {
  pinAState_left = digitalRead(encodePinA_left);
  pinBState_left = digitalRead(encodePinB_left);
  pinCState_left = digitalRead(encodePinC_left);

  if (pinAState_left == 0 && pinBState_left == 1 && pinCState_left == 1) {                       //value:=1
    if (pinAStateOld_left == 0 && pinBStateOld_left == 1 && pinCStateOld_left == 0)              //value:=5 // CW
      Encoderpos_left ++;
    if (pinAStateOld_left == 0 && pinBStateOld_left == 0 && pinCStateOld_left == 1)              //value:=3 // CCW
      Encoderpos_left --;
  }

  if (pinAState_left == 0 && pinBState_left == 0 && pinCState_left == 1) {                       //value:=3
    if (pinAStateOld_left == 0 && pinBStateOld_left == 1 && pinCStateOld_left == 1)              //value:=1 // CW
      Encoderpos_left ++;
    if (pinAStateOld_left == 1 && pinBStateOld_left == 0 && pinCStateOld_left == 1)              //value:=2 // CCW
      Encoderpos_left --;
  }

  if (pinAState_left == 1 && pinBState_left == 0 && pinCState_left == 1) {                       //value:=2
    if (pinAStateOld_left == 0 && pinBStateOld_left == 0 && pinCStateOld_left == 1)              //value:=3 // CW
      Encoderpos_left ++;
    if (pinAStateOld_left == 1 && pinBStateOld_left == 0 && pinCStateOld_left == 0)              //value:=6 // CCW
      Encoderpos_left --;
  }

  if (pinAState_left == 1 && pinBState_left == 0 && pinCState_left == 0) {                       //value:=6
    if (pinAStateOld_left == 1 && pinBStateOld_left == 0 && pinCStateOld_left == 1)              //value:=2 // CW
      Encoderpos_left ++;
    if (pinAStateOld_left == 1 && pinBStateOld_left == 1 && pinCStateOld_left == 0)              //value:=4 // CCW
      Encoderpos_left --;
  }

  if (pinAState_left == 1 && pinBState_left == 1 && pinCState_left == 0) {                       //value:=4
    if (pinAStateOld_left == 1 && pinBStateOld_left == 0 && pinCStateOld_left == 0)              //value:=6 // CW
      Encoderpos_left ++;
    if (pinAStateOld_left == 0 && pinBStateOld_left == 1 && pinCStateOld_left == 0)              //value:=5 // CCW
      Encoderpos_left --;
  }

  if (pinAState_left == 0 && pinBState_left == 1 && pinCState_left == 0) {                       //value:=5
    if (pinAStateOld_left == 1 && pinBStateOld_left == 1 && pinCStateOld_left == 0)              //value:=4 // CW
      Encoderpos_left ++;
    if (pinAStateOld_left == 0 && pinBStateOld_left == 1 && pinCStateOld_left == 1)              //value:=1 // CCW
      Encoderpos_left --;
  }

  pinAStateOld_left = pinAState_left;
  pinBStateOld_left = pinBState_left;
  pinCStateOld_left = pinCState_left;
}
void doEncoder_right() {
  pinAState_right = digitalRead(encodePinA_right);
  pinBState_right = digitalRead(encodePinB_right);
  pinCState_right = digitalRead(encodePinC_right);

  if (pinAState_right == 0 && pinBState_right == 1 && pinCState_right == 1) {                       //value:=1
    if (pinAStateOld_right == 0 && pinBStateOld_right == 1 && pinCStateOld_right == 0)              //value:=5 // CW
      Encoderpos_right ++;
    if (pinAStateOld_right == 0 && pinBStateOld_right == 0 && pinCStateOld_right == 1)              //value:=3 // CCW
      Encoderpos_right --;
  }

  if (pinAState_right == 0 && pinBState_right == 0 && pinCState_right == 1) {                       //value:=3
    if (pinAStateOld_right == 0 && pinBStateOld_right == 1 && pinCStateOld_right == 1)              //value:=1 // CW
      Encoderpos_right ++;
    if (pinAStateOld_right == 1 && pinBStateOld_right == 0 && pinCStateOld_right == 1)              //value:=2 // CCW
      Encoderpos_right --;
  }

  if (pinAState_right == 1 && pinBState_right == 0 && pinCState_right == 1) {                       //value:=2
    if (pinAStateOld_right == 0 && pinBStateOld_right == 0 && pinCStateOld_right == 1)              //value:=3 // CW
      Encoderpos_right ++;
    if (pinAStateOld_right == 1 && pinBStateOld_right == 0 && pinCStateOld_right == 0)              //value:=6 // CCW
      Encoderpos_right --;
  }

  if (pinAState_right == 1 && pinBState_right == 0 && pinCState_right == 0) {                       //value:=6
    if (pinAStateOld_right == 1 && pinBStateOld_right == 0 && pinCStateOld_right == 1)              //value:=2 // CW
      Encoderpos_right ++;
    if (pinAStateOld_right == 1 && pinBStateOld_right == 1 && pinCStateOld_right == 0)              //value:=4 // CCW
      Encoderpos_right --;
  }

  if (pinAState_right == 1 && pinBState_right == 1 && pinCState_right == 0) {                       //value:=4
    if (pinAStateOld_right == 1 && pinBStateOld_right == 0 && pinCStateOld_right == 0)              //value:=6 // CW
      Encoderpos_right ++;
    if (pinAStateOld_right == 0 && pinBStateOld_right == 1 && pinCStateOld_right == 0)              //value:=5 // CCW
      Encoderpos_right --;
  }

  if (pinAState_right == 0 && pinBState_right == 1 && pinCState_right == 0) {                       //value:=5
    if (pinAStateOld_right == 1 && pinBStateOld_right == 1 && pinCStateOld_right == 0)              //value:=4 // CW
      Encoderpos_right ++;
    if (pinAStateOld_right == 0 && pinBStateOld_right == 1 && pinCStateOld_right == 1)              //value:=1 // CCW
      Encoderpos_right --;
  }

  pinAStateOld_right = pinAState_right;
  pinBStateOld_right = pinBState_right;
  pinCStateOld_right = pinCState_right;
}

void revFromMCU_left()
{
  if (Serial1.available()) {
    if (Serial1.read() == '{') {
      Serial1.readBytes(getData_left, 8);
      if (getData_left[7] == '}') {
        checksum_left = (0x55 ^ getData_left[0] ^ getData_left[1] ^ getData_left[2] ^ getData_left[3] ^ getData_left[4] ^ getData_left[5]);
        if (checksum_left == getData_left[6]) {
          for (int i = 0; i < 3; i ++) {
            MotorData_left[i].Byte.L = getData_left[2 * i];
            MotorData_left[i].Byte.H = getData_left[2 * i + 1];
          }
          Serial.println(String("MCU Vq_left=") + " " + \
                         String(MotorData_left[0].Data) + " " + \
                         String("MCU Vd_left=") + " " + \
                         String(MotorData_left[1].Data) + " " + \
                         String("MCU rds=") + " " + \
                         String(MotorData_left[2].Data));
        }
      }
    }
  }
}
void revFromMCU_right() {
  if (Serial1.available()) {
    if (Serial1.read() == '{') {
      Serial1.readBytes(getData_right, 8);
      if (getData_right[7] == '}') {
        checksum_right = (0x55 ^ getData_right[0] ^ getData_right[1] ^ getData_right[2] ^ getData_right[3] ^ getData_right[4] ^ getData_right[5]);
        if (checksum_right == getData_right[6]) {
          for (int i = 0; i < 3; i ++) {
            MotorData_right[i].Byte.L = getData_right[2 * i];
            MotorData_right[i].Byte.H = getData_right[2 * i + 1];
          }
          Serial.println(String("MCU Vq_right=") + " " + \
                         String(MotorData_right[0].Data) + " " + \
                         String("MCU Vd_right=") + " " + \
                         String(MotorData_right[1].Data) + " " + \
                         String("MCU rds=") + " " + \
                         String(MotorData_right[2].Data));
        }
      }
    }
  }
}
void sendCmd_left()
{
	  if (Vq_left >= Vq_MAX) 	    Vq_left = Vq_MAX;
	  else if (Vq_left <= Vq_MIN)    Vq_left = Vq_MIN;

//	  if ((abs(Vq_left) <= VD_ENABLE_LIMITE) && (Vq_left != 0))
//	  {
//		Vd_left = VD_ENABLE_VALUE;                                                         // The Vd always be postive value, even Vq is a negative value
//		digitalWrite(51, HIGH);                                                       // turn the LED on (HIGH is the voltage level)
//	  }
//	  else
//	  {
//		Vd_left = 0;
//		digitalWrite(51, LOW);                                                        // turn the LED off by making the voltage LOW
//	  }

	  Vd_left = 0;
	  sendData[1] = highByte(Vq_left);
	  sendData[2] = lowByte(Vq_left);
	  sendData[3] = highByte(Vd_left);
	  sendData[4] = lowByte(Vd_left);
	  sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);

	  Serial1.write(sendData, 7);
}
void sendCmd_right()
{
	  if (Vq_right >= Vq_MAX) 	    Vq_right = Vq_MAX;
	  else if (Vq_right <= Vq_MIN)    Vq_right = Vq_MIN;

//	  if ((abs(Vq_right) <= VD_ENABLE_LIMITE) && (Vq != 0))
//	  {
//		Vd_right = VD_ENABLE_VALUE;                                                         // The Vd always be postive value, even Vq is a negative value
//		digitalWrite(51, HIGH);                                                       // turn the LED on (HIGH is the voltage level)
//	  }
//	  else
//	  {
//		Vd_right = 0;
//		digitalWrite(51, LOW);                                                        // turn the LED off by making the voltage LOW
//	  }

	  Vd_right = 0;
	  sendData[1] = highByte(Vq_right);
	  sendData[2] = lowByte(Vq_right);
	  sendData[3] = highByte(Vd_right);
	  sendData[4] = lowByte(Vd_right);
	  sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);

	  Serial1.write(sendData, 7);
}

void getMotorData_left() {
  EncodeDiff_left = Encoderpos_left - EncoderposPre_left;
  omega_actual_left = ((EncodeDiff_left) * (1000 / dT)) * 2 * PI / (CPR);

  //  Encoderpos=0;//recount
  EncoderposPre_left = Encoderpos_left;
  Serial.println(String("EncodeDiff_left=") + " " + String(EncodeDiff_left));
  Serial.println(String("dT=") + " " + String(dT));
}
void getMotorData_right() {
  EncodeDiff_right = Encoderpos_right - EncoderposPre_right;
  omega_actual_right = ((EncodeDiff_right) * (1000 / dT)) * 2 * PI / (CPR);

  //  Encoderpos=0;//recount
  EncoderposPre_right = Encoderpos_right;
  Serial.println(String("EncodeDiff_right=") + " " + String(EncodeDiff_right));
  Serial.println(String("dT=") + " " + String(dT));
}

ros::NodeHandle nh;

andbot1dot2::WheelFb vel_msg;
ros::Publisher p("feedback_wheel_angularVel", &vel_msg);
ros::Subscriber<andbot1dot2::WheelCmd> s("cmd_wheel_volt", cmd_wheel_voltCb);
ros::ServiceServer<andbot1dot2::DriverStateRequest, andbot1dot2::DriverStateResponse> service("DriverState_service", &DriverState_service_callback);

void setup(){

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(s);
    nh.advertise(p);
    nh.advertiseService(service);

    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);

    pinMode(51, OUTPUT);                                                           //for VD_ENABLE_VALUE check

    //left wheel
    pinMode(encodePinA_left, INPUT);
    pinMode(encodePinB_left, INPUT);
    pinMode(encodePinC_left, INPUT);
    digitalWrite(encodePinA_left, HIGH);                                                 //turn on pullup resistor
    digitalWrite(encodePinB_left, HIGH);                                                 //turn on pullup resistor
    digitalWrite(encodePinC_left, HIGH);                                                 //turn on pullup resistor

    pinAStateOld_left = digitalRead(encodePinA_left);
    pinBStateOld_left = digitalRead(encodePinB_left);
    pinCStateOld_left = digitalRead(encodePinC_left);

    attachInterrupt(0, doEncoder_left, CHANGE);                                          //encoder pin on interrupt 0 - pin 2
    attachInterrupt(1, doEncoder_left, CHANGE);                                          //encoder pin on interrupt 1 - pin 3
    attachInterrupt(2, doEncoder_left, CHANGE);

    //right wheel
    pinMode(encodePinA_right, INPUT);
    pinMode(encodePinB_right, INPUT);
    pinMode(encodePinC_right, INPUT);
    digitalWrite(encodePinA_right, HIGH);                                                 //turn on pullup resistor
    digitalWrite(encodePinB_right, HIGH);                                                 //turn on pullup resistor
    digitalWrite(encodePinC_right, HIGH);                                                 //turn on pullup resistor

    pinAStateOld_right = digitalRead(encodePinA_right);
    pinBStateOld_right = digitalRead(encodePinB_right);
    pinCStateOld_right = digitalRead(encodePinC_right);

    attachInterrupt(0, doEncoder_right, CHANGE);                                          //encoder pin on interrupt 0 - pin 2
    attachInterrupt(1, doEncoder_right, CHANGE);                                          //encoder pin on interrupt 1 - pin 3
    attachInterrupt(2, doEncoder_right, CHANGE);
}

void loop()
{
    readFeadback_angularVel_L();
    readFeadback_angularVel_R();

    if ((millis() - lastMilli) >= LOOPTIME)
    {
        dT = millis() - lastMilli;
        lastMilli = millis();

        vel_msg.speed1 = omega_left_actual;
        vel_msg.speed2 = omega_right_actual;
        vel_msg.current1 = 0.0;
        vel_msg.current2 = 0.0;
        p.publish(&vel_msg);
    }
    nh.spinOnce();
}
