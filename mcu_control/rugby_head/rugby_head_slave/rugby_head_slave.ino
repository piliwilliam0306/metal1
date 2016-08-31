#include <EEPROM.h>
//RS485
#define RS485Transmit    HIGH
#define RS485Receive     LOW 
#define RS485Mode 8
//#define SLAVE   3
//motor
#define encoderPinA 2
#define encoderPinB 3
#define MotorPin0 6
#define MotorPin1 5
#define MC33926Enable 7
#define LOOPTIME 10
#define PRINTTIME 100
#define HOME_SPEED_PWM 150
#define CW 1  //L0 L2 L3 R2 R3
#define CCW -1  //L1 R0 R1
//#define AXIS_TYPE CW

#define pan 0
//#define tilt 1


#if defined (pan)
  #define SLAVE 0
  #define AXIS_TYPE CW
#elif defined (tilt)
  #define SLAVE 1
  #define AXIS_TYPE CCW
#endif
    
// STATE MACHINE
#define INIT_STATE 0
#define HOME_STATE 1
#define HOME_ZONE_STATE 2
#define WORKING_RANGE_STATE 3
#define END_ZONE_STATE 4
#define MAX_PWM 250
double rad_tick = 0.00019;//(double)6.28/32767; //resolution
double degree_tick = 0.011;//(double)360/32767; //resolution

int state=WORKING_RANGE_STATE;
int home_switch=0;
int offset_flag=0;

double CPR = 64;
double gear_ratio = 810;

volatile long Encoderpos = 0;
volatile long unknownvalue = 0;

volatile int lastEncoded = 0;
unsigned long lastMilli = 0;
unsigned long lastPrint = 0;
long dT = 0;

double cmdPwm=0;
double cmdPos_ROS=0,anglePos_ROS=0; //degree
double pos_offset=0;
int pos_offset_v=0;
double max_working_rage=0;
double cmdPos_Joint=0,anglePos_Joint=0,anglePos_Joint_last=0,angleSpeed_Joint;
double Kp=20;
/*
byte ID = 0;  //address ID
byte P = 1;  //address proportional 
byte I = 3;  //address intergral
byte D = 5;  //address derivative
byte C_L = 7;  //address current limit byte

byte Kp = 0;
byte Ki = 0;
byte Kd = 0;
byte SLAVE = 0; 
unsigned int Current_Limit = 0;
*/
//current sense
int analogPin = A0;
unsigned int current = 0;
byte current_send = 0;

void setup() 
{
  TCCR0B = TCCR0B & B11111000 | B00000010;
  pinMode(encoderPinA, INPUT); digitalWrite(encoderPinA, HIGH); // turn on pullup resistor
  pinMode(encoderPinB, INPUT); digitalWrite(encoderPinB, HIGH);

  attachInterrupt(0, doEncoder, CHANGE);  // encoder pin on interrupt 0 - pin 2
  attachInterrupt(1, doEncoder, CHANGE);
  
  pinMode(MotorPin0, OUTPUT); pinMode(MotorPin1, OUTPUT); 
  pinMode(MC33926Enable, OUTPUT); digitalWrite(MC33926Enable, HIGH);
  
  pinMode(RS485Mode, OUTPUT); digitalWrite(RS485Mode, RS485Receive); //DE,RE=LOW, RX enabled
  Serial.begin (1000000);   // TO MAX485
  //Serial.begin (115200);   // for test
  /*Kp = EEPROM.read(P);  Ki = EEPROM.read(I);  Kd = EEPROM.read(D);
  SLAVE = EEPROM.read(ID);
  Current_Limit = EEPROM.read(C_L) * 10;*/
}

void loop() 
{
  //readCmd();
  //readTest();
  get_cmd_pos();
  if((millis()-lastMilli) >= LOOPTIME)   
     {                                  
        dT = millis()-lastMilli;
        lastMilli = millis();
        control_loop();
        angleSpeed_Joint = double((anglePos_Joint-anglePos_Joint_last)/0.01);
        anglePos_Joint_last=anglePos_Joint;
        //printTemple();
     }
}

void control_loop(){

    int HomeValue=0;//analogRead(HomePin);
    int EndValue=0;//analogRead(EndPin);

    //CREATE cmdPos_Joint
    pos_offset=pos_offset+0.01*pos_offset_v;
    
    cmdPos_Joint=cmdPos_ROS+pos_offset;

    //GET ENC
    get_angle_from_enc();

    //CONTROL STRATERGY
    switch(state){

      case INIT_STATE:
            send_cmd_to_motor(0);
            if(home_switch==1)  state=HOME_STATE;
            break;
       
      case HOME_ZONE_STATE:
            if(home_switch==1)  go_joint_pos_ros(0);
            else{
              if(cmdPos_ROS>anglePos_ROS) {
                    go_joint_pos_ros(cmdPos_ROS);
                    state=WORKING_RANGE_STATE;
              }
              else  go_joint_pos_ros(0);
            }
            break;
 
      case WORKING_RANGE_STATE:
            if(home_switch==1)
              state=HOME_STATE;
            else {
              if(HomeValue>650) state=HOME_ZONE_STATE;
              else if (EndValue>650){
                  max_working_rage=anglePos_ROS;
                  state=END_ZONE_STATE;
                } 
              else  go_joint_pos_ros(cmdPos_ROS);
            }
            break; 
      } 
  }
  
void printTemple(){
        Serial.print(" cmdPos_ROS: "); Serial.print(cmdPos_ROS); 
        Serial.print(" cmdPos_Joint: ");Serial.print(cmdPos_ROS); 
        Serial.print(" anglePos_ROS: "); Serial.print(anglePos_ROS); 
        Serial.print(" anglePos_Joint: ");Serial.print(anglePos_Joint); 
        Serial.print(" cmdPwm: ");Serial.print(cmdPwm); 
        Serial.print(" dT: ");Serial.print(dT);
        Serial.println("");     
  
  }
