//This code uses direct port manipulation which only works on Arduino Mega 2560
//#define ANDBOT 1
//#define RUGBY 2
#define ANGELBOT 3

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <Bump.h>
#include <Sonar.h>
#include <WheelCmd.h>
#include <WheelFb.h>
#include <avr/io.h>
#include <DriverState.h>

#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#if defined (ANDBOT)
	#define MaxSpeed 10.96
#elif (ANGELBOT)
	#define MaxSpeed 31
#else
	#define MaxSpeed 31
#endif

const int TrigPin1 = 30;  //PC7
const int TrigPin2 = 31;  //PC6
const int TrigPin3 = 32;  //PC5
const int TrigPin4 = 33;  //PC4
const int TrigPin5 = 34;  //PC3
const int TrigPin6 = 35;  //PC2
const int TrigPin7 = 36;  //PC1
const int TrigPin8 = 37;  //PC0

const int EchoPin1 = 42;  //PL7
const int EchoPin2 = 43;  //PL6
const int EchoPin3 = 44;  //PL5
const int EchoPin4 = 45;  //PL4
const int EchoPin5 = 46;  //PL3
const int EchoPin6 = 47;  //PL2
const int EchoPin7 = 48;  //PL1
const int EchoPin8 = 49;  //PL0

#define LOOPTIME        100
#define BOOLTIME        1000 //1 Hz publish rate for cliff and bump sensor
#define PUBLISHRATE 	(1/25*1000)

double omega_left_target = 0.0;
double omega_right_target = 0.0;
double omega_left_actual = 0;
double omega_right_actual = 0;
unsigned long lastMilli = 0;
unsigned long lastBool = 0;
unsigned long PrePubMilli = 0;

long dT = 0;

bool bump1_reading;
bool bump2_reading;
bool bump3_reading;
bool bump4_reading;
bool cliff1_reading;
bool cliff2_reading;
bool cliff3_reading;
bool cliff4_reading;
//Max.Distance(cm) = 200cm
#define TimeOut 5000//TimeOut = Max.Distance(cm) * 58

unsigned int current_left = 0;
unsigned int current_right = 0;
bool driver_mode;

// variables for MIMO closed loop control
double vel_ref = 0.0;
double vel_fb = 0.0;
double omega_ref = 0.0;
double omega_fb = 0.0;
double u_sum = 0.0;
double u_diff = 0.0;
double omega_fb_right = 0.0;
double omega_fb_left = 0.0;

// variables for controllers
double sum_error_vel = 0.0;
double sum_error_omega = 0.0;

double WheelRadius = 0.0375, WheelSeparation = 0.247;

//declarations
void cmd_velCallback(const geometry_msgs::Twist &twist_aux);
void DriverState_service_callback(const angelbot::DriverStateRequest& req, angelbot::DriverStateResponse& res);
void WheelCmdModeCallback(const angelbot::WheelCmd &WheelCmd);
void LoopRateCallback(const std_msgs::Float32 &rate);

/* ************  declarations for ROS usages *****************************************/

/*  define  ROS node and topics */

ros::NodeHandle nh;
using angelbot::DriverState;
bool set_; 

angelbot::WheelFb WheelFb_msgs;
angelbot::WheelCmd WheelCmd_msgs;
geometry_msgs::Twist VelFb_msgs;
ros::Publisher feedback_wheel_angularVel_pub("angelbot/feedback_wheel_angularVel", &WheelFb_msgs);
ros::Publisher cmd_wheel_volt_pub("angelbot/cmd_wheel_volt", &WheelCmd_msgs);
ros::Publisher feedbackVel_pub("andbot1dot2/feedbackVel",&VelFb_msgs);
ros::Subscriber<angelbot::WheelCmd> s("angelbot/cmd_wheel_angularVel",WheelCmdModeCallback);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/andbot1dot2/cmd_vel", cmd_velCallback);

angelbot::Bump bump_msg;
ros::Publisher pub_bump("bump", &bump_msg);

angelbot::Sonar sonar_msg;
ros::Publisher pub_sonar( "sonar", &sonar_msg);
/* ************  End of declarations for ROS usages ****************/

void sendCmd_wheel_angularVel_L()
{
  int left_target_send = int(omega_left_target/(double(MaxSpeed)/32767)); //convert rad/s to 16 bit integer to send
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
  int right_target_send = int(omega_right_target/(double(MaxSpeed)/32767)); //convert rad/s to 16 bit integer to send
  byte buf[4];
  buf[0] = '{'; //send start byte
  buf[1] = highByte(right_target_send); //send high byte
  buf[2] = lowByte(right_target_send);  //send low byte
  if (driver_mode == true) buf[3] = '}'; //send stop byte motor on
  if (driver_mode == false) buf[3] = '|'; //send stop byte motor off
  Serial1.write(buf, sizeof(buf));
}

//callback
void WheelCmdModeCallback(const angelbot::WheelCmd& msg)
{

}

void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
  double u_left = 0.0;
  double u_right = 0.0;
  double volt_friction_compensation = 0.7;

  vel_ref = twist_aux.linear.x;
  omega_ref = twist_aux.angular.z;

//  u_sum = vel_controller(vel_ref,vel_fb) + vel_ref * FeedForward_vel;
//  u_diff = omega_controller(omega_ref, omega_fb) + omega_ref * FeedForward_omega ;

//  if(u_sum >= Umax_volt)              u_sum = Umax_volt;
//  else if(u_sum <= Umin_volt) u_sum = Umin_volt;
//  else;
//
//  if(u_diff >= Umax_volt)             u_diff = Umax_volt;
//  else if(u_diff <= Umin_volt)        u_diff = Umin_volt;
//  else;
//
//  u_right = (u_sum + u_diff) / 2 ;
//  u_left = (u_sum - u_diff) / 2 ;

  // friction compensation
//  if (vel_ref != 0.0 || omega_ref != 0.0)
//  {
//        if (u_right < 0.0)      u_right = u_right - volt_friction_compensation;
//        else                            u_right = u_right + volt_friction_compensation;
//
//        if (u_left < 0.0)       u_left = u_left - volt_friction_compensation;
//        else                            u_left = u_left + volt_friction_compensation;
//  }
//  else;

	u_right = (vel_ref + (omega_ref*WheelSeparation)/2)/ WheelRadius;
	u_left = (vel_ref - (omega_ref*WheelSeparation)/2) / WheelRadius;

	//for publish
  WheelCmd_msgs.speed1 = u_left ;
  WheelCmd_msgs.speed2 = u_right;

  omega_left_target = u_left;
  omega_right_target = u_right;
  //driver_mode = msg.driverstate;
  sendCmd_wheel_angularVel_L();
  sendCmd_wheel_angularVel_R();
}
void FbVelCal(const angelbot::WheelFb &wheel)
{
  omega_fb_left = wheel.speed1;
  omega_fb_right = wheel.speed2;

  //mobile robot kinematics transformation: differential drive
  vel_fb = double(WheelRadius) / 2 * (omega_fb_right + omega_fb_left);
  omega_fb = double(WheelRadius) / double(WheelSeparation) * (omega_fb_right - omega_fb_left);

  VelFb_msgs.linear.x = vel_fb;
  VelFb_msgs.angular.z = omega_fb;
}

void setup() 
{
  TCCR0B = TCCR0B & B11111000 | B00000010; 
  //set baud rate for rosserial
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(feedbackVel_pub);
  nh.advertise(cmd_wheel_volt_pub);
  nh.advertise(feedback_wheel_angularVel_pub);
  nh.advertise(pub_sonar);
  nh.advertise(pub_bump);

  Serial2.begin (115200);  //left
  Serial1.begin (115200);  //right
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

		// for publish
		WheelFb_msgs.speed1 = omega_left_actual;
		WheelFb_msgs.speed2 = omega_right_actual;
		FbVelCal(WheelFb_msgs);

		WheelFb_msgs.current1 = current_left;
		WheelFb_msgs.current2 = current_right;
	}
	else;

	if((millis()-PrePubMilli) >= PUBLISHRATE)
	{
		PrePubMilli = millis();
		feedback_wheel_angularVel_pub.publish(&WheelFb_msgs);
		feedbackVel_pub.publish(&VelFb_msgs);
		cmd_wheel_volt_pub.publish(&WheelCmd_msgs);
	}
	else;

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
          
          //sonar
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
    else;
    	nh.spinOnce();
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
            omega_left_actual = double (actual_receive * (double(MaxSpeed)/32767)); //convert received 16 bit integer to actual speed
            //max current is 20400mA, 255 * 80 = 20400mA
            current_left = (rCS_L * 80); 
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
          omega_right_actual = double (actual_receive * (double(MaxSpeed)/32767)); //convert received 16 bit integer to actual speed
          current_right = rCS_R * 80;
        }  
     }
  }   
}

