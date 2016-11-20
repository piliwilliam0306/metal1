/*
 * mimo_base_controller
 */

#include"mbed.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

uint8_t highByte (int in)
{
    uint8_t out = in >> 8;
    return out;
}

uint8_t lowByte (int in)
{
    uint8_t out = in & 0X00FF;
    return out;
}
//////////////////////////////
//Hardware Configuration Start
//////////////////////////////
DigitalOut led = LED1;
Serial Serial1(PA_9, PA_10);
Serial Serial3(PC_10, PC_11);
class NewHardware : public MbedHardware
{
  public:
  NewHardware():MbedHardware(USBTX, USBRX, 57600){};  
};
//////////////////////////////
//Hardware Configuration END
//////////////////////////////

//ROS init Start
ros::NodeHandle_<NewHardware>  nh;
std_msgs::String str_msg;
//ROS init END
 
//global variable Start
//cmd
double cmd_model_vel_x=0.0;
double cmd_model_omega_z=0.0;
double cmd_wheel_omega_left = 0.0;
double cmd_wheel_omega_right = 0.0;
//mimo model
double Kp = 30;
double Ki = 5;
double us;
double ud;
//PWM 
double cmd_wheel_pwm_left = 0.0;
double cmd_wheel_pwm_right = 0.0;
//feedback
double feedback_model_vel_x = 0.0;
double feedback_model_omega_z = 0.0;
double feedback_wheel_omega_left = 0;
double feedback_wheel_omega_right = 0;
//global variable END 



void model_PI_controller_update(){
    us = (cmd_model_vel_x - feedback_model_vel_x)*Kp;
    ud = (cmd_model_omega_z - feedback_model_omega_z)*Ki;
    
    cmd_wheel_pwm_left = 0.5*(us-ud)*double(255/12);
    if (cmd_wheel_pwm_left>=255)
        cmd_wheel_pwm_left=255;
    else if (cmd_wheel_pwm_left<=-255)
        cmd_wheel_pwm_left=-255;

    cmd_wheel_pwm_right = 0.5*(us+ud)*double(255/12);
    if (cmd_wheel_pwm_right>=255)
        cmd_wheel_pwm_right=255;
    else if (cmd_wheel_pwm_right<=-255)
        cmd_wheel_pwm_right=-255;
}

void sendCmd_wheel_PWM_left()
{
  int left_target_send = int(cmd_wheel_pwm_left/0.0077822); //convert rad/s to 16 bit integer to send 
  char sT_L = '{'; //send start byte
  uint8_t sH_L = highByte(left_target_send); //send high byte
  uint8_t sL_L = lowByte(left_target_send);  //send low byte
  char sP_L = '}'; //send stop byte
  Serial1.putc(sT_L); Serial1.putc(sH_L); Serial1.putc(sL_L); Serial1.putc(sP_L);
}

void sendCmd_wheel_PWM_right()
{
  int right_target_send = int(cmd_wheel_pwm_right/0.0077822); //convert rad/s to 16 bit integer to send 
  char sT_L = '{'; //send start byte
  uint8_t sH_L = highByte(right_target_send); //send high byte
  uint8_t sL_L = lowByte(right_target_send);  //send low byte
  char sP_L = '}'; //send stop byte
  Serial3.putc(sT_L); Serial3.putc(sH_L); Serial3.putc(sL_L); Serial3.putc(sP_L);
}

void cmd_velCallback(const geometry_msgs::Vector3& cmdVel_msg)
{   
    cmd_model_vel_x = cmdVel_msg.x;
    cmd_model_omega_z = cmdVel_msg.z;
    model_PI_controller_update();
    sendCmd_wheel_PWM_left();
    sendCmd_wheel_PWM_right();

}


ros::Subscriber<geometry_msgs::Vector3> cmd_vel_sub("/andbot/cmd_vel", cmd_velCallback);

ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";


int main() {
    Serial1.baud(1000000); 
    Serial3.baud(1000000); 
    nh.initNode();
    nh.advertise(chatter);
    nh.subscribe(cmd_vel_sub);   
    
    while (1) {
                        
        //Serial1.putc('S');                
        led = !led;
        str_msg.data = hello;
        chatter.publish( &str_msg );
        nh.spinOnce();
        wait_ms(10);
    }
}


