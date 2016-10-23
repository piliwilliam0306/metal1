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

#define Relay_pin_left 52    //  digital output
#define Relay_pin_right 53    //  digital output
Metro systemwarmup = Metro(5000) ; //(msec)
bool systemReadyFlag = false;

char val;
char commandArray_L[3];
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

#define LOOPTIME 100

#define Vq_MAX 1000//520
#define Vq_MIN -1000//-520
const double MAX_AngularSpeed = 450 / 60 * 2 * 3.1415926; // nominal rotation speed: 450 rpm

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

void sendCmd_wheel_volt_L()
{
    if(volt_left_target>Vq_MAX)           volt_left_target=Vq_MAX;
    else if(volt_left_target<Vq_MIN)      volt_left_target=Vq_MIN;

    left_target_send = int(volt_left_target / (double(Vq_MAX)/double(32767)));   //convert received 16 bit integer to actual voltage => Vq_MAX/32767

    //transmit command to the lower level mega board.
    char sT_L = '{'; //send start byte
    byte sH_L = highByte(left_target_send);
    byte sL_L = lowByte(left_target_send);
    char sP_L = '}';
    Serial2.write(sT_L); Serial2.write(sH_L); Serial2.write(sL_L); Serial2.write(sP_L);
}


void sendCmd_wheel_volt_R()
{
    if(volt_right_target>Vq_MAX)          volt_right_target=Vq_MAX;
    else if(volt_right_target<Vq_MIN)     volt_right_target=Vq_MIN;

    right_target_send = int(volt_right_target / (double(Vq_MAX)/double(32767)));   //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904

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

ros::NodeHandle nh;

andbot1dot2::WheelFb vel_msg;
ros::Publisher p("feedback_wheel_angularVel", &vel_msg);
ros::Subscriber<andbot1dot2::WheelCmd> s("cmd_wheel_volt", cmd_wheel_voltCb);
ros::ServiceServer<andbot1dot2::DriverStateRequest, andbot1dot2::DriverStateResponse> service("DriverState_service", &DriverState_service_callback);

void setup(){

    pinMode(Relay_pin_left, OUTPUT);
    pinMode(Relay_pin_right, OUTPUT);

    nh.getHardware()->setBaud(115200);
    nh.initNode();
    nh.subscribe(s);
    nh.advertise(p);
    nh.advertiseService(service);

    Serial.println("Please wait for system warming up ...");
    while(systemwarmup.check() == false);
    digitalWrite(Relay_pin_left, HIGH);  // power on the LED
    digitalWrite(Relay_pin_right, HIGH);
    Serial.println("Warmup finish");

    Serial1.begin(115200);
    Serial2.begin(115200);
    Serial3.begin(115200);
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
