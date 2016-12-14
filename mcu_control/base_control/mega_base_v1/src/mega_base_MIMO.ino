#define ANDBOT 1
//#define RUGBY 2

#include <ros.h>
//#include <andbot/Bump.h>
//#include <andbot/Sonar.h>
#include <andbot/WheelCmd.h>
#include <andbot/WheelFb.h>
#include <avr/io.h>
//#include <andbot/DriverState.h>
#include <andbot/Battery.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>

#define LOOPTIME        100
#define SONARTIME       200
#define BOOLTIME        1000 //1 Hz publish rate for cliff and bump sensor

double omega_left_target = 0.0;
double omega_right_target = 0.0;
double omega_left_actual = 0;
double omega_right_actual = 0;
unsigned long lastMilli = 0;
unsigned long lastBool = 0;
unsigned long lastSonar = 0;

long dT = 0;


//Max.Distance(cm) = 200cm
#define TimeOut 15000//TimeOut = Max.Distance(cm) * 58

unsigned int current_left = 0;
unsigned int current_right = 0;
bool driver_mode = false;

int Vin = 0;           // variable to store the value read
int Cin = 0;
unsigned int current = 0;           // variable to store the value read
byte capacity = 0;
double voltage = 0;

ros::NodeHandle nh;
//using andbot::DriverState;
bool set_; 

andbot::WheelFb wheel_msg;
ros::Publisher p("feedback_wheel_angularVel", &wheel_msg);

andbot::Bump bump_msg;
ros::Publisher pub_bump("bump", &bump_msg);

andbot::Sonar sonar_msg;
ros::Publisher pub_sonar( "sonar", &sonar_msg);
/*
andbot::Battery battery_msg;
ros::Publisher pub_battery("battery", &battery_msg);
*/

std_msgs::UInt8 battery_msg;
ros::Publisher pub_battery("battery", &battery_msg);

//callback
void messageCb(const andbot::WheelCmd& msg)
{
  omega_left_target = msg.speed1;  
  omega_right_target = msg.speed2;
  driver_mode = msg.driverstate;
  sendCmd_wheel_angularVel_L();
  sendCmd_wheel_angularVel_R();
}

ros::Subscriber<andbot::WheelCmd> s("cmd_wheel_angularVel",messageCb);
/*
void callback(const DriverState::Request & req, DriverState::Response & res)
{
  driver_mode = req.driverstate;  
  res.driverstate = driver_mode;
  sendCmd_wheel_angularVel_L();
  sendCmd_wheel_angularVel_R();
}
*/
//ros::ServiceServer<DriverState::Request, DriverState::Response> server("DriverState",&callback);

void setup() 
{
  //TCCR0B = TCCR0B & B11111000 | B00000010; 
  //set baud rate for rosserial
  nh.getHardware()->setBaud(1000000); 
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
  nh.advertise(pub_sonar);
  nh.advertise(pub_bump);
  nh.advertise(pub_battery);
  //nh.advertiseService(server);

  Serial2.begin (1000000);  //left
  Serial1.begin (1000000);  //right
  DDRA &= ~0b11111111;  //set DDRA register as input for bump and cliff sensors
  DDRL &= ~0b11111111;  //set DDRL register as input for Echo pins
  DDRC |= 0b11111111;  //set DDRC register as output for Trig pins
}

void loop() 
{
  readFeadback_angularVel_L();
  readFeadback_angularVel_R();   
  
  if((millis()-lastMilli) >= LOOPTIME)   
       {                                    // enter tmed loop
          dT = millis()-lastMilli;
          lastMilli = millis();

          wheel_msg.speed1 = omega_left_actual;
          wheel_msg.speed2 = omega_right_actual;
          wheel_msg.current1 = current_left;
          wheel_msg.current2 = current_right;
          p.publish(&wheel_msg);
       }     
    
    if((millis()-lastBool) >= BOOLTIME)   
       {
          lastBool = millis();

          //bump sensors
          bump1_reading = (PINA & (1<<PA1));
          bump_msg.bump1 = !bump1_reading;
          bump2_reading = (PINA & (1<<PA3));
          bump_msg.bump2 = !bump2_reading;
          bump3_reading = (PINA & (1<<PA5));
          bump_msg.bump3 = !bump3_reading;
          bump4_reading = (PINA & (1<<PA7));
          bump_msg.bump4 = !bump4_reading;
          
          //cliff sensors
          cliff1_reading = (PINA & (1<<PA0));
          bump_msg.cliff1 = !cliff1_reading;
          cliff2_reading = (PINA & (1<<PA2));
          bump_msg.cliff2 = !cliff2_reading;
          cliff3_reading = (PINA & (1<<PA4));
          bump_msg.cliff3 = !cliff3_reading;
          cliff4_reading = (PINA & (1<<PA6));
          bump_msg.cliff4 = !cliff4_reading;
          pub_bump.publish(&bump_msg);
          
          batteryStatus();
          /*battery_msg.capacity = capacity;
          battery_msg.current = current;*/
          battery_msg.data = capacity;
          pub_battery.publish(&battery_msg);
        }  
        
  if((millis()-lastSonar) >= SONARTIME)   
       {                                    // enter tmed loop
          lastSonar = millis();

          sonar_msg.sonar1 = ping(TrigPin1,EchoPin1);
          sonar_msg.sonar2 = ping(TrigPin2,EchoPin2);
          sonar_msg.sonar3 = ping(TrigPin3,EchoPin3);
          sonar_msg.sonar4 = ping(TrigPin4,EchoPin4);
          sonar_msg.sonar5 = ping(TrigPin5,EchoPin5);
          sonar_msg.sonar6 = ping(TrigPin6,EchoPin6);
          sonar_msg.sonar7 = ping(TrigPin7,EchoPin7);
          sonar_msg.sonar8 = ping(TrigPin8,EchoPin8);
          pub_sonar.publish(&sonar_msg);
       }
  nh.spinOnce();
}

void batteryStatus()
{
  Vin = analogRead(A0);
  //delay(20);
  //Vin = analogRead(A0);
  
  voltage = Vin * 0.01642228739;
  if (voltage > 16.66)  capacity = 100;
  else if (voltage <= 14)  capacity = 0;
  else                     capacity = (voltage-14)/0.028;

  //Cin = analogRead(A1);
  //voltage at 0A = vcc/2, nominally 2.5VDC, 509 count by measurement
  //current = ((double(Cin-510)*5)/1024)/0.066*1000; //convert to mA
}

uint8_t ping(int TrigPin, int EchoPin) 
{
  unsigned long duration;
  uint8_t cm;
  digitalWrite( TrigPin, LOW );
  delayMicroseconds( 2 );
  digitalWrite( TrigPin, HIGH );
  delayMicroseconds( 10 );
  digitalWrite( TrigPin, LOW );
  duration = pulseIn( EchoPin, HIGH,TimeOut);
  cm = (duration/2) / 29.1;
  return cm; 
}

void readFeadback_angularVel_L()
{
  int actual_receive;
  if (Serial2.available() >= 5) 
  {
    char rT_L = (char)Serial2.read(); //read actual speed from Uno
    if(rT_L == '{')
      {
        char commandArray_L[4];
        Serial2.readBytes(commandArray_L,4);
        byte rH_L = commandArray_L[0];
        byte rL_L = commandArray_L[1];
        byte rCS_L = commandArray_L[2];
        char rP_L = commandArray_L[3];
        if(rP_L == '}')       
          {
            actual_receive = (rH_L << 8) + rL_L; 
            omega_left_actual = double (actual_receive * (MaxSpeed/32767)); //convert received 16 bit integer to actual speed
            //max current is 10200mA, 255 * 40 = 10200mA
            current_left = (rCS_L * 40); 
          }
      }   
  }
}

void readFeadback_angularVel_R()
{
  int actual_receive;
  if (Serial1.available() >= 5) 
  {  
    char rT_R = (char)Serial1.read(); //read actual speed from Uno
    if(rT_R == '{')
     {
       char commandArray_R[4];
       Serial1.readBytes(commandArray_R,4);
       byte rH_R = commandArray_R[0];
       byte rL_R = commandArray_R[1];
       byte rCS_R = commandArray_R[2];
       char rP_R = commandArray_R[3];
       if(rP_R == '}')          
        {
          actual_receive = (rH_R << 8) + rL_R; 
          omega_right_actual = double (actual_receive * (MaxSpeed/32767)); //convert received 16 bit integer to actual speed
          current_right = rCS_R * 40;
        }  
     }
  }   
}

void sendCmd_wheel_angularVel_L()
{
  int left_target_send = int(omega_left_target/(MaxSpeed/32767)); //convert rad/s to 16 bit integer to send
  byte buf[4];
  buf[0] = '{'; //send start byte
  buf[1] = highByte(left_target_send); //send high byte
  buf[2] = lowByte(left_target_send);  //send low byte
  if (driver_mode == true)  buf[3] = '}'; //send stop byte motor on
  if (driver_mode == false)  buf[3] = '|'; //send stop byte motor off
  Serial2.write(buf, sizeof(buf));
}

void sendCmd_wheel_angularVel_R()
{
  int right_target_send = int(omega_right_target/(MaxSpeed/32767)); //convert rad/s to 16 bit integer to send
  byte buf[4];
  buf[0] = '{'; //send start byte
  buf[1] = highByte(right_target_send); //send high byte
  buf[2] = lowByte(right_target_send);  //send low byte
  if (driver_mode == true) buf[3] = '}'; //send stop byte motor on
  if (driver_mode == false) buf[3] = '|'; //send stop byte motor off
  Serial1.write(buf, sizeof(buf));
}



