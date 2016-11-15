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
#include <DDWheel.h>

union Data_Setting {
  struct _ByteSet {
    byte L;
    byte H;
  } Byte;
  int Data;
};

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

// motor parameters
int CPR = 90;                                                                     //encoder count per revolution
int gear_ratio = 1;
int actual_send = 0;
const double MAX_AngularSpeed = 47.1238898 ;//  450 / 60 * 2 * PI => DD motor nominal rotation speed: 450 rpm

int target_receive = 0;

#define Volt_MAX 12
#define Volt_MIN -12
#define Vq_MAX 1000 //  ~8V
#define Vq_MIN -1000 // ~-8V
const double Vq_formatRatio=1499/12; //convert voltage to Vq voltage format ( 12 (max voltage) : 1499 (Vq format max) )

bool driverEn;

unsigned long lastMilli = 0;
long dT = 0;
int left_actual_receive = 0;
int left_target_send = 0;
int right_actual_receive = 0;
int right_target_send = 0;

union Data_Setting MotorData_left[3];
//byte getData_left[8];
//byte sendData_left[7] = {123, 0, 0, 0, 0, 0, 125};
//byte sendDataStop_left[7] = {123, 0, 0, 0, 0, 85, 125};

//int Vq_left = 0, Vd_left = 0, checksum_left = 0;

#define left 0
#define right 1

//encoder pin assignment
//left wheel
#define enc_pinA_left 2
#define enc_pinB_left 3
#define enc_pinC_left 21
//right wheel
#define enc_pinA_right 5
#define enc_pinB_right 6
#define enc_pinC_right 7

DDWheel Wheel_left(enc_pinA_left,enc_pinB_left,enc_pinC_left,left);
DDWheel Wheel_right(enc_pinA_right,enc_pinB_right,enc_pinC_right,right);

void readCmd_wheel_volt()
{
        if (rT == '{')
        {
//                  byte commandArray[3];
//                  Serial3.readBytes(commandArray, 3);
//                  byte rH = commandArray[0];
//                  byte rL = commandArray[1];
//                  char rP = commandArray[2];

                  if (rP == '}')
                  {
                        target_receive = (rH << 8) + rL;
                        vol_target = double (target_receive * double(Volt_MAX)/double(32767));          //convert received 16 bit integer to actual voltage => Vq_MAX/32767
                        if(WHEEL_SELECT==0) //left wheel
                        vol_target = -vol_target;
                  }
        }

}    //end of if (Serial3.available())

void sendCmd_wheel_volt_L()
{
    if(volt_left_target>Volt_MAX)           volt_left_target=Volt_MAX;
    else if(volt_left_target<Volt_MIN)      volt_left_target=Volt_MIN;

    //left_target_send = int(volt_left_target / (double(Volt_MAX)/double(32767)));   //convert received 16 bit integer to actual voltage => Vq_MAX/32767

//    //transmit command to the lower level mega board.
//    char sT_L = '{'; //send start byte
//    byte sH_L = highByte(left_target_send);
//    byte sL_L = lowByte(left_target_send);
//    char sP_L = '}';
//    Serial2.write(sT_L); Serial2.write(sH_L); Serial2.write(sL_L); Serial2.write(sP_L);
}


void sendCmd_wheel_volt_R()
{
    if(volt_right_target>Volt_MAX)          volt_right_target=Volt_MAX;
    else if(volt_right_target<Volt_MIN)     volt_right_target=Volt_MIN;

    right_target_send = int(volt_right_target / (double(Volt_MAX)/double(32767)));   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904

//    //transmit command to the lower level mega board.
//    char sT_R = '{';
//    byte sH_R = highByte(right_target_send);
//    byte sL_R = lowByte(right_target_send);
//    char sP_R = '}';
//    Serial1.write(sT_R); Serial1.write(sH_R); Serial1.write(sL_R); Serial1.write(sP_R);
}

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

void cmd_wheel_voltCb(const andbot1dot2::WheelCmd& msg)
{
    volt_left_target = msg.speed1;
    volt_right_target = msg.speed2;
    sendCmd_wheel_volt_L();
    sendCmd_wheel_volt_R();
}

//void readFeadback_angularVel_L()
//{
//    if (Serial2.available() >= 4)
//    {
//        char rT_L = (char)Serial2.read();
//        if (rT_L == '{')
//        {
//            char commandArray_L[3];
//            Serial2.readBytes(commandArray_L,3);
//            byte rH_L = commandArray_L[0];
//            byte rL_L = commandArray_L[1];
//            char rP_L = commandArray_L[2];
//            if (rP_L == '}')
//            {
//                left_actual_receive =(rH_L << 8) + rL_L;
//                omega_left_actual = double (left_actual_receive * (MAX_AngularSpeed/double(32767)));   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
//            }
//        }
//    }
//}

//void readFeadback_angularVel_R()
//{
//    if (Serial1.available() >= 4)
//    {
//        char rT_R = (char)Serial1.read();
//        if (rT_R == '{')
//        {
//            char commandArray_R[3];
//            Serial1.readBytes(commandArray_R, 3);
//            byte rH_R = commandArray_R[0];
//            byte rL_R = commandArray_R[1];
//            char rP_R = commandArray_R[2];
//            if (rP_R == '}')
//            {
//                right_actual_receive = (rH_R << 8) + rL_R;
//                omega_right_actual = double (right_actual_receive * (MAX_AngularSpeed/double(32767)));   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
//            }
//        }
//    }
//}

ros::NodeHandle nh;

andbot1dot2::WheelFb vel_msg;
ros::Publisher p("feedback_wheel_angularVel", &vel_msg);
ros::Subscriber<andbot1dot2::WheelCmd> s("cmd_wheel_volt", cmd_wheel_voltCb);
ros::ServiceServer<andbot1dot2::DriverStateRequest, andbot1dot2::DriverStateResponse> service("DriverState_service", &DriverState_service_callback);
cmd_vel_sub = n1.subscribe("/andbot1dot2/cmd_vel", 10, cmd_velCallback);

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

    Wheel_left.Init(); //left wheel
    Wheel_right.Init();//right wheel
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

  vel_ref = twist_aux.linear.x;
  omega_ref = twist_aux.angular.z;

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

  cmd_wheel_volt_pub.publish(wheel);
}

void feedback_wheel_angularVelCallback(const andbot1dot2::WheelFb &wheel)
{
  geometry_msgs::Twist twist_aux;
  omega_fb_left = wheel.speed1;
  omega_fb_right  = wheel.speed2;

  //mobile robot kinematics transformation: differential drive
  vel_fb = wheelRadius / 2 * (omega_fb_right + omega_fb_left);
  omega_fb = wheelRadius / wheelSeparation * (omega_fb_right - omega_fb_left);

  twist_aux.linear.x = vel_fb;
  twist_aux.angular.z = omega_fb;
  feedback_Vel_pub.publish(twist_aux);
}

void loop()
{
    readFeadback_angularVel_L();
    readFeadback_angularVel_R();

    if ((millis() - lastMilli) >= LOOPTIME)
    {
        dT = millis() - lastMilli;
        lastMilli = millis();

        Wheel_left.FbMotorData();
        Wheel_right.FbMotorData();


        if (vol_target == 0)
        {
                Vq = 0;
                //sum_error = 0;
        }
        else
        {
                Vq = vol_target * Vq_formatRatio;
        }

        Wheel_left.sendVoltCmd();
        Wheel_right.sendVoltCmd();

        vel_msg.speed1 = fb_omega_left;
        vel_msg.speed2 = fb_omega_right;
        vel_msg.current1 = 0.0;
        vel_msg.current2 = 0.0;
        p.publish(&vel_msg);
    }
    nh.spinOnce();
}
