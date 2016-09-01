
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
#define AXIS_TYPE CW

#define MAX_PWM 250
double rad_tick = 0.00019;//(double)6.28/32767; //resolution
double degree_tick = 0.011;//(double)360/32767; //resolution

double CPR = 48;
double gear_ratio = 500;

volatile long Encoderpos = 0;
volatile long unknownvalue = 0;

volatile int lastEncoded = 0;
unsigned long lastMilli = 0;
unsigned long lastPrint = 0;
long dT = 0;

double cmdPwm=0;
double cmdPos_ROS=0,anglePos_ROS=0; //degree
double max_working_rage=0;
double cmdPos_Joint=0,anglePos_Joint=0;
double Kp=80;//sfsg

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
  
  //pinMode(RS485Mode, OUTPUT); digitalWrite(RS485Mode, RS485Receive); //DE,RE=LOW, RX enabled
  Serial.begin (57600);   // TO MAX485

}

void loop() 
{
  //readCmd();
  readTest();
  //get_cmd_pos();
  if((millis()-lastMilli) >= LOOPTIME)   
     {                                  
        dT = millis()-lastMilli;
        lastMilli = millis();
        control_loop();
        printTemple();

     }
}

void control_loop(){

    get_angle_from_enc();
    go_joint_pos_ros(cmdPos_ROS);

  }

void readTest()
{
  if (Serial.available() >= 4)  cmdPos_ROS = Serial.parseFloat();
}

  
void printTemple(){
        //Serial.print(" vPlus: ");Serial.print(vPlus);
        Serial.print(" cmdPos_ROS: ");Serial.print(cmdPos_ROS);
        Serial.print(" anglePos_Joint: ");Serial.print(anglePos_Joint);
        Serial.print(" cmdPwm: ");Serial.print(cmdPwm);
        Serial.print(" dT: ");Serial.print(dT);
        Serial.println("");     
  
  }
