/*   
   Author: Zach qoogood1234@gmail.com

   20160427 from joint_position_VHN_promini to joint_RS485
   20160503 ADD TCCR0B = TCCR0B & B11111000 | B00000010; 
            modify ~/arduino-1.6.5-r5/hardware/arduino/avr/cores/arduino/wiring.c
            #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256))
   

*/

#include <SoftwareSerial.h>
#define L298N 0
#define VNH5019 1
#define DRIVER VNH5019
//#define AXIS_TYPE OUTWARD
#define CW 1 //L0 L2 L3 R2 R3
#define CCW -1 //L1 R0 R1
#define AXIS_TYPE CCW 
#define HOME_SPEED_PWM 150 //150

#define LOOPTIME 10
//MOTOR PWM PIN ASSIGNMENT

//l298n driver board
#define MotorPin0 10 //l298n
#define MotorPin1 11 //l298n

//VNH driver board 
#define MotorINA 4
#define MotorINB 5
#define MotorPWM 6

//ENCODER PIN ASSIGNMENT
#define encoderPinA 2
#define encoderPinB 3

// STATE MACHINE
#define INIT_STATE 0
#define HOME_STATE 1
#define HOME_ZONE_STATE 2
#define WORKING_RANGE_STATE 3
#define END_ZONE_STATE 4
#define MAX_PWM 255
double rad_tick=(double)62.83/32767; //10rev 2^15
double degree_tick=(double)3600/32767; //


int state=WORKING_RANGE_STATE;
int home_switch=0;
int offset_flag=0;


char commandArray[4];
byte sT = 0;  //send start byte
byte sA = 0;  //send device address + command byte
byte sH = 0;  //send high byte
byte sL = 0;  //send low byte
byte sP = 0;  //send stop byte

byte rT = 0;  //receive start byte
byte rA = 0;  //receive device address + command byte
byte rH = 0;  //receive high byte
byte rL = 0;  //receive low byte
byte rP = 0;  //receive stop byte

#define RS485Transmit    HIGH
#define RS485Receive     LOW 

//#define master 0
//#define slave 1
//#define slave 2
//#define slave 3
#define slave 4

//#define slave 5
//#define slave 6
//#define slave 7
//#define slave 8

unsigned long lastMilli = 0;                    // loop timing 
long dT = 0;

double cmdPwm=0;
double cmdPos_ROS=0,anglePos_ROS=0; //degree
double pos_offset=0;
int pos_offset_v=0;
double max_working_rage=0;
double cmdPos_Joint=0,anglePos_Joint=0,anglePos_Joint_last=0,angleSpeed_Joint;
double Kp=10;
//ENC STATE
int pinAState = 0;int pinAStateOld = 0;int pinBState = 0;int pinBStateOld = 0;

volatile long encoderPos = 0;
volatile long unknownvalue = 0;

//HOME
#define HomePin 0
#define EndPin 1


void setup() { 
 TCCR0B = TCCR0B & B11111000 | B00000010;

    pinMode(MotorPin0, OUTPUT);
    pinMode(MotorPin1, OUTPUT);

    pinMode(MotorINA, OUTPUT);
    pinMode(MotorINB, OUTPUT);
    pinMode(MotorPWM, OUTPUT);  
      
    pinMode(encoderPinA, INPUT); 
    digitalWrite(encoderPinA, HIGH); // turn on pullup resistor
    pinMode(encoderPinB, INPUT); 
    digitalWrite(encoderPinB, HIGH); // turn on pullup resistor

    attachInterrupt(0, doEncoder, CHANGE); // encoder pin on interrupt 0 - pin 2
    attachInterrupt(1, doEncoder, CHANGE);
    digitalWrite(8, RS485Receive); //DE,RE=LOW, RX enabled
    Serial.begin (1000000);

} 

void loop(){
  //GET CMD FROM MEGA
  readCmd();
  if((millis()-lastMilli) >= LOOPTIME)   
                    {                                    // enter tmed loop
                        dT = millis()-lastMilli;
                        lastMilli = millis();
                        
                        control_loop(); 
                        angleSpeed_Joint = double((anglePos_Joint-anglePos_Joint_last)/0.01);
                        anglePos_Joint_last=anglePos_Joint;
                        //printTemple();
                    } 
//printTemple();
}

void control_loop(){
    /////WRITE ENC to mega
    
    //READ HOME SENSOR VALUE
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
            if(home_switch==1)
              state=HOME_STATE;
            break;
       
      case HOME_ZONE_STATE:
            if(home_switch==1)
              go_joint_pos_ros(0);
            else{
              if(cmdPos_ROS>anglePos_ROS) {
                    go_joint_pos_ros(cmdPos_ROS);
                    state=WORKING_RANGE_STATE;
              }
              else
                    go_joint_pos_ros(0);
            }
            break;


            
      case WORKING_RANGE_STATE:
            if(home_switch==1)
              state=HOME_STATE;
            else {
              if(HomeValue>650) {
                  state=HOME_ZONE_STATE;
              }
              else if (EndValue>650){
                  max_working_rage=anglePos_ROS;
                  state=END_ZONE_STATE;
                } 
                else
                {
                  //PWM testing 正轉
                  //digitalWrite(MotorINA,1);digitalWrite(MotorINB,0);analogWrite(MotorPWM,255); 
                  //PWM testing 反轉
                  //digitalWrite(MotorINA,0);digitalWrite(MotorINB,1);analogWrite(MotorPWM,255); 
                  //position control
                  go_joint_pos_ros(cmdPos_ROS);            
                }              
            }

           
            break;
 
      } 
  }



void go_joint_pos_ros(double target_pos_ros) {
      double target_pos_joint=target_pos_ros+pos_offset;
      cmdPwm=Kp*(target_pos_joint-anglePos_Joint);
  if(AXIS_TYPE == CCW){
    if(DRIVER==L298N)
      send_cmd_to_motor(-cmdPwm);
    else if(DRIVER==VNH5019)
      send_cmd_to_motor_VNH5019(-cmdPwm);
  }
  else{
    if(DRIVER==L298N)
      send_cmd_to_motor(cmdPwm);
    else if(DRIVER==VNH5019)
      send_cmd_to_motor_VNH5019(cmdPwm);  
     }
  }


void readCmd()
{
  //digitalWrite(8, RS485Receive); //DE,RE=LOW, RX enabled
  if (Serial.available() >= 10) 
  {
    char rT = (char)Serial.read(); //read target speed from mega
          if(rT == '{')//start byte
            {
              char commandArray[9];
              Serial.readBytes(commandArray,9);
              byte rA=commandArray[0]; //device address + command
              byte rH=commandArray[1]; //high byte
              byte rL=commandArray[2]; //low byte
              byte rH1=commandArray[3]; //high byte
              byte rL1=commandArray[4]; //low byte
              byte rH2=commandArray[5]; //high byte
              byte rL2=commandArray[6]; //low byte
              byte rH3=commandArray[7]; //high byte
              char rP=commandArray[8]; //stop byte
              if(rP=='}' && rA==slave)         
                {
                  cmdPos_ROS = ((rH<<8)+rL) * degree_tick;  //convert received 16 bit integer to actual speed
                  Serial.write(rH);
                  Serial.write(rL);
                }
                
            }
  }         
}


void get_angle_from_enc() {
  
  if(AXIS_TYPE == CCW){
          anglePos_Joint=1*encoderPos*0.00694;//(double)360/(64*810*1);      
          anglePos_ROS=anglePos_Joint-pos_offset;
  }
  else{
          anglePos_Joint=-1*encoderPos*0.00694;//(double)360/(64*810*1);      
          anglePos_ROS=anglePos_Joint-pos_offset;  
  }
}



void send_cmd_to_motor(int cmdpwm) {

    int vPlus=0,vMinus=0;
    if(cmdpwm>=0)
    {
      vPlus=cmdpwm;
      vMinus=0;
    }
    else if(cmdpwm<0)
     {
      vPlus=0;
      vMinus=-cmdpwm;
    }   
    
     if(vPlus>=MAX_PWM)
        vPlus=MAX_PWM;
     else if(vPlus<-MAX_PWM)
          vPlus=-MAX_PWM;
          
          
     if(vMinus>=MAX_PWM)
        vMinus=MAX_PWM;
     else if(vMinus<-MAX_PWM)
          vMinus=-MAX_PWM;      
    analogWrite(MotorPin1,vPlus);
    analogWrite(MotorPin0,vMinus);
 
}


void send_cmd_to_motor_VNH5019(int cmdpwm) {

    int dir;
    if (cmdpwm>=0)
      dir=1;
    else
      dir=-1;
      
    cmdpwm=abs(cmdpwm);

    
    if(cmdpwm>=MAX_PWM)
      cmdpwm=MAX_PWM;
    if (dir==1){
      digitalWrite(MotorINA,1);
      digitalWrite(MotorINB,0);
      analogWrite(MotorPWM,cmdpwm); 
    }
    else{
      digitalWrite(MotorINA,0);
      digitalWrite(MotorINB,1);
      analogWrite(MotorPWM,cmdpwm);      
      }
}

void doEncoder() {
    // encoderPos++;
    pinAState = digitalRead(2);
    pinBState = digitalRead(3);

    if (pinAState == 0 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 0) // forward
    encoderPos ++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // reverse
    encoderPos --;
    if (pinAStateOld == 1 && pinBStateOld == 1) // unknown
    unknownvalue ++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // unknown
    unknownvalue ++;
    }
    if (pinAState == 0 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 0) // forward
    encoderPos ++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // reverse
    encoderPos --;
    if (pinAStateOld == 1 && pinBStateOld == 0) // unknown
    unknownvalue ++;
    if (pinAStateOld == 0 && pinBStateOld == 1) // unknown
    unknownvalue ++;
    }
    if (pinAState == 1 && pinBState == 1) {
    if (pinAStateOld == 0 && pinBStateOld == 1) // forward
    encoderPos ++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // reverse
    encoderPos --;
    if (pinAStateOld == 0 && pinBStateOld == 0) // unknown
    unknownvalue ++;
    if (pinAStateOld == 1 && pinBStateOld == 1) // unknown
    unknownvalue ++;
    }

    if (pinAState == 1 && pinBState == 0) {
    if (pinAStateOld == 1 && pinBStateOld == 1) // forward
    encoderPos ++;
    if (pinAStateOld == 0 && pinBStateOld == 0) // reverse
    encoderPos --;
    if (pinAStateOld == 0 && pinBStateOld == 1) // unknown
    unknownvalue ++;
    if (pinAStateOld == 1 && pinBStateOld == 0) // unknown
    unknownvalue ++;
    }
    pinAStateOld = pinAState;
    pinBStateOld = pinBState;
}



void printTemple(){
        Serial.print(" cmdPos_ROS: "); Serial.print(cmdPos_ROS); 
        Serial.print(" anglePos_ROS: ");Serial.print(anglePos_ROS); 
        //Serial.print(" anglePos_ROS: "); Serial.print(anglePos_ROS); 
        //Serial.print(" anglePos_Joint: ");Serial.print(anglePos_Joint); 
        //Serial.print(" angleSpeed_Joint: ");Serial.print(angleSpeed_Joint); 
        //Serial.print(" cmdPwm: ");Serial.print(cmdPwm); 
        Serial.println("");     
  
  }

