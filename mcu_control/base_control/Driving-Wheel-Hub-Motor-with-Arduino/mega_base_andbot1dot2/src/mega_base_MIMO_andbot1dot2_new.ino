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
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <WheelCmd.h>
#include <WheelFb.h>
#include <DriverState.h>
#include <Metro.h>
#include "DDWheel.h"

// robot param
double wheelRadius = 0.085, wheelSeparation = 0.432;

#define LOOPTIME 40//100
#define rate 25
unsigned long lastMilli = 0;
long dT = 0;

#define Volt_MAX 12
#define Volt_MIN -12
#define Vq_MAX 1000 //  ~8V
#define Vq_MIN -1000 // ~-8V

DDWheel::SetdqCmdLimit BLDC_InputLimit = {Volt_MAX,Volt_MIN,Vq_MAX,Vq_MIN};

// motor parameters
int CPR = 90;                                                                     //encoder count per revolution
int gear_ratio = 1;
const double MAXAngularSpeed = 47.1238898 ;//  450 / 60 * 2 * PI => DD motor nominal rotation speed: 450 rpm

#define MotorDefault \
{\
	90,\
	1,\
	1,\
}

DDWheel::SetMotorParam BLDC_DDWheel = {CPR,gear_ratio,MAXAngularSpeed};//MotorDefault;//CPR,gear_ratio,MAXAngularSpeed};

//encoder pin assignment
//left wheel
#define enc_pinA_left 2
#define enc_pinB_left 3
#define enc_pinC_left 21
//right wheel
#define enc_pinA_right 5
#define enc_pinB_right 6
#define enc_pinC_right 7

DDWheel::SetENCPin left_enc = {enc_pinA_left,enc_pinB_left,enc_pinC_left};
DDWheel::SetENCPin right_enc = {enc_pinA_right,enc_pinB_right,enc_pinC_right};

#define Axis_left 0
#define Axis_right 1

//cmd ref
#define VqVdMode 0
#define VqIdMode 1
#define IdIqMode 2

DDWheel::SetCtrlParam Ctrl_left = {Axis_left, VqVdMode};
DDWheel::SetCtrlParam Ctrl_right = {Axis_right, VqVdMode};

DDWheel Wheel_left(&left_enc,&BLDC_DDWheel,&Ctrl_left,&BLDC_InputLimit);
DDWheel Wheel_right(&right_enc,&BLDC_DDWheel,&Ctrl_right,&BLDC_InputLimit);

bool driverEn;

// variables for MIMO closed loop control
double vel_ref=0.0;
double vel_fb = 0.0;
double omega_ref = 0.0;
double omega_fb = 0.0;
double u_sum = 0.0;
double u_diff = 0.0;
double omega_fb_right = 0.0;
double omega_fb_left = 0.0;

// varibles for controllers
double sum_error_vel = 0.0;
double sum_error_omega = 0.0;

void DriverState_service_callback(const andbot1dot2::DriverStateRequest& req, andbot1dot2::DriverStateResponse& res)
{
    driverEn = req.driverstate;
    if (driverEn == true)
    {
      res.driverstate = true;
      Wheel_left.Enable();
      Wheel_right.Enable();
//      Serial1.write("m", 1);
//      Serial2.write("m", 1);
    }
    else
    {
      res.driverstate = false;
      Wheel_left.Disable();
      Wheel_right.Disable();
//      Serial1.write("k", 1);
//      Serial2.write("k", 1);
    }
    Serial.print("From Client");
    Serial.println(req.driverstate,DEC);
    Serial.print("Server says");
    Serial.print(res.driverstate,DEC);
}
double vel_controller(double targetValue, double currentValue)
{
  //static double last_error = 0;
  //long dT = 1/rate;
  double error;
  double iTerm;
  double iTerm_Umax = 6;
  double iTerm_Umin = -6;
  double pidTerm ;
  //double sum_error ;
  // PI control
  double Kp = 14.9624;
  double Ki = 21.3962;

  error = targetValue - currentValue;

  sum_error_vel = sum_error_vel + error * (1/rate);
  iTerm = Ki * sum_error_vel;

  if (iTerm >= iTerm_Umax)        	iTerm = iTerm_Umax;
  else if (iTerm<= iTerm_Umin)		iTerm = iTerm_Umin;

  pidTerm = Kp * error + iTerm;

  //ROS_INFO("In vel_loop, we get error:[%f], sum_error:[%f] and pidTerm:[%f]", error, sum_error_vel, pidTerm);

  //saturation protection
//  if (pidTerm >= 10)        	constrained_pidTerm = 10;
//  else if (pidTerm <= -10)  	constrained_pidTerm = -10;
//  else 	                        constrained_pidTerm = pidTerm ;

  return pidTerm;
}
double omega_controller(double targetValue, double currentValue)
{
  //static double last_error = 0;
  //long dT = 1/rate;
  double error;
  double iTerm;
  double iTerm_Umax = 6;
  double iTerm_Umin = -6;
  double pidTerm;
  //double sum_error;
  double Kp = 4.4157;
  double Ki = 5.8287;

  error = targetValue - currentValue;

  sum_error_omega = sum_error_omega + error * (1/rate);
  iTerm = Ki * sum_error_omega;

  if (iTerm >= iTerm_Umax)        	iTerm = iTerm_Umax;
  else if (iTerm<= iTerm_Umin)		iTerm = iTerm_Umin;

  pidTerm = Kp * error + iTerm;

  //ROS_INFO("In omega_loop, we get error:[%f], sum_error:[%f] and pidTerm:[%f]", error, sum_error_omega, pidTerm);
//  if (pidTerm >= 10)  		constrained_pidTerm = 10;
//  else if (pidTerm <= -10) 	constrained_pidTerm = -10;
//  else  	                constrained_pidTerm = pidTerm ;

  return pidTerm;
}
// MIMO control loop
void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
  andbot1dot2::WheelCmd wheel;
  geometry_msgs::Twist twist = twist_aux;
  double u_left = 0.0;
  double u_right = 0.0;
  double volt_friction_compensation = 0.7;
  double FeedForward_vel = 6.313/1.054; // from model
  double FeedForward_omega = 4.9887/3.2393; //from model

  vel_ref = twist.linear.x;
  omega_ref = twist.angular.z;

  u_sum = vel_controller(vel_ref,vel_fb) + vel_ref * FeedForward_vel;
  u_diff = omega_controller(omega_ref, omega_fb) + omega_ref * FeedForward_omega ;

//  if(u_sum >= Umax_volt)              u_sum = Umax_volt;
//  else if(u_sum <= Umin_volt) u_sum = Umin_volt;
//  else;
//
//  if(u_diff >= Umax_volt)             u_diff = Umax_volt;
//  else if(u_diff <= Umin_volt)        u_diff = Umin_volt;
//  else;

  u_right = (u_sum + u_diff) / 2 ;
  u_left = (u_sum - u_diff) / 2 ;

  // friction compensation
  if (vel_ref != 0.0 || omega_ref != 0.0)
  {
        if (u_right < 0.0)      u_right = u_right - volt_friction_compensation;
        else                            u_right = u_right + volt_friction_compensation;

        if (u_left < 0.0)       u_left = u_left - volt_friction_compensation;
        else                            u_left = u_left + volt_friction_compensation;
  }
  else;

  wheel.speed1 = u_left ;
  wheel.speed2 = u_right;
  //cmd_wheel_volt_pub.publish(wheel);

  Wheel_left.cmd_volt = u_left;
  Wheel_right.cmd_volt = u_right;
}

void feedback_wheel_angularVelCal(const andbot1dot2::WheelFb &wheel)
{
  geometry_msgs::Twist twist_aux;
  omega_fb_left = wheel.speed1;
  omega_fb_right = wheel.speed2;

  //mobile robot kinematics transformation: differential drive
  vel_fb = wheelRadius / 2 * (omega_fb_right + omega_fb_left);
  omega_fb = wheelRadius / wheelSeparation * (omega_fb_right - omega_fb_left);

  twist_aux.linear.x = vel_fb;
  twist_aux.angular.z = omega_fb;
  //feedback_Vel_pub.publish(twist_aux);
}

/* ************  declarations for ROS usages *****************************************/

/*  define  ROS node and topics */
ros::NodeHandle nh;

andbot1dot2::WheelFb velFb_msg;
andbot1dot2::WheelCmd velCmd_msg;
ros::Publisher feedback_wheel_angularVel_pub("feedback_wheel_angularVel",&velFb_msg);
ros::Publisher cmd_wheel_volt_pub("cmd_wheel_volt", &velCmd_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/andbot1dot2/cmd_vel", cmd_velCallback);
ros::ServiceServer<andbot1dot2::DriverStateRequest, andbot1dot2::DriverStateResponse> service("DriverState_service", &DriverState_service_callback);

/* ************  End of declarations for ROS usages ****************/

void setup(){

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.advertise(cmd_wheel_volt_pub);
    nh.advertise(feedback_wheel_angularVel_pub);
    nh.subscribe(cmd_vel_sub);
    nh.advertiseService(service);

    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);

    pinMode(51, OUTPUT);                                                           //for VD_ENABLE_VALUE check

    Wheel_left.Init(); //left wheel
    Wheel_right.Init();//right wheel

	attachInterrupt(0, ISR_left, CHANGE);                                          //encoder pin on interrupt 0 - pin 2
	attachInterrupt(1, ISR_left, CHANGE);                                          //encoder pin on interrupt 1 - pin 3
	attachInterrupt(2, ISR_left, CHANGE);
	attachInterrupt(3, ISR_right, CHANGE);                                          //encoder pin on interrupt 0 - pin 2
	attachInterrupt(4, ISR_right, CHANGE);                                          //encoder pin on interrupt 1 - pin 3
	attachInterrupt(5, ISR_right, CHANGE);
}

void loop()
{

    if ((millis() - lastMilli) >= LOOPTIME)
    {
        dT = millis() - lastMilli;
        lastMilli = millis();

        Wheel_left.FbMotorData(dT);
        Wheel_right.FbMotorData(dT);

        velFb_msg.speed1 = Wheel_left.fb_omega;
        velFb_msg.speed2 = Wheel_right.fb_omega;

        feedback_wheel_angularVelCal(velFb_msg);

        Wheel_left.sendVoltCmd();
        Wheel_right.sendVoltCmd();

//        vel_msg.speed1 = fb_omega_left;
//        vel_msg.speed2 = fb_omega_right;
//        vel_msg.current1 = 0.0;
//        vel_msg.current2 = 0.0;
//        feedback_wheel_angularVel_pub.publish(&vel_msg);
    }
    nh.spinOnce();
}

void ISR_left()
{
	Wheel_left.doEncoder();
}
void ISR_right()
{
	Wheel_right.doEncoder();
}
