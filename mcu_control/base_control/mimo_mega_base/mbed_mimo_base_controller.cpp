
/*
 * mimo_base_controller
 */

#include"mbed.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define MaxSpeed 10.96

Ticker timer1;

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
Serial pc(USBTX, USBRX);
Serial Serial1(PA_9, PA_10);
Serial Serial3(PC_10, PC_11);
class NewHardware : public MbedHardware
{
  public:
  NewHardware():MbedHardware(USBTX, USBRX, 1000000){};  
};

//function prototype start
void cmd_velCallback(const geometry_msgs::Twist& cmdVel_msg);
void readFeadback_angularVel_L();
void model_PI_controller_update();
void sendCmd_wheel_PWM_left();
void sendCmd_wheel_PWM_right();
void inverse_diff_model();
void forward_diff_model();
//function prototype End

//////////////////////////////
//Hardware Configuration END
//////////////////////////////

 
//global variable Start
//Mechanical Param
double wheelSeparation = 0.393;
double wheelRadius = 0.078;

//cmd
double cmd_model_vel_x=0.0;
double cmd_model_omega_z=0.0;
double cmd_wheel_omega_left = 0.0;
double cmd_wheel_omega_right = 0.0;

//mimo model
double feedforwardGain = 30;
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

//ROS init Start
ros::NodeHandle_<NewHardware>  nh;
std_msgs::String str_msg;
geometry_msgs::Twist feedback_vel_msg;
geometry_msgs::Vector3 feedback_wheel_angularVel_msg;
geometry_msgs::Vector3 cmd_wheel_angularVel_msg;
geometry_msgs::Vector3 cmd_wheel_PWM_msg;

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/andbot/cmd_vel", cmd_velCallback);
ros::Publisher cmd_wheel_angularVel_pub("/andbot/cmd_wheel_angularVel", &cmd_wheel_angularVel_msg);
ros::Publisher cmd_wheel_PWM_pub("/andbot/cmd_wheel_PWM", &cmd_wheel_PWM_msg);
ros::Publisher feedback_wheel_angularVel_pub("/andbot/feedback_wheel_angularVel", &feedback_wheel_angularVel_msg);
ros::Publisher feedback_vel_pub("/andbot/feedback_vel", &feedback_vel_msg);

ros::Publisher chatter("chatter", &str_msg);
//ROS init END


void cmd_velCallback(const geometry_msgs::Twist& cmdVel_msg)
{  
    cmd_model_vel_x = cmdVel_msg.linear.x;
    cmd_model_omega_z = cmdVel_msg.angular.z;
}

void inverse_diff_model(){
    cmd_wheel_omega_left = (2*cmd_model_vel_x - cmd_model_omega_z * wheelSeparation) / 2 / wheelRadius;
    cmd_wheel_omega_right =(2*cmd_model_vel_x + cmd_model_omega_z * wheelSeparation) / 2 / wheelRadius;
 
}
void forward_diff_model(){
    feedback_model_vel_x = wheelRadius * (feedback_wheel_omega_left + feedback_wheel_omega_right) / 2;
    feedback_model_omega_z = wheelRadius * (feedback_wheel_omega_right - feedback_wheel_omega_left) / wheelSeparation;
 
}
void model_PI_controller_update(){
    us = (cmd_model_vel_x - feedback_model_vel_x)*Kp;
    ud = (cmd_model_omega_z - feedback_model_omega_z)*Ki;
    
    cmd_wheel_pwm_left = 0.5*(us-ud)*double(255/12)+cmd_wheel_omega_left*double(30);
    if (cmd_wheel_pwm_left>=255)
        cmd_wheel_pwm_left=255;
    else if (cmd_wheel_pwm_left<=-255)
        cmd_wheel_pwm_left=-255;

    cmd_wheel_pwm_right = 0.5*(us+ud)*double(255/12)+cmd_wheel_omega_right*double(30);
    if (cmd_wheel_pwm_right>=255)
        cmd_wheel_pwm_right=255;
    else if (cmd_wheel_pwm_right<=-255)
        cmd_wheel_pwm_right=-255;
          
}

void sendCmd_wheel_PWM_left()
{
  //cmd_wheel_pwm_left=100;
  int left_target_send = int(cmd_wheel_pwm_left/0.0077822); //convert rad/s to 16 bit integer to send 
  char sT_L = '{'; //send start byte
  uint8_t sH_L = highByte(left_target_send); //send high byte
  uint8_t sL_L = lowByte(left_target_send);  //send low byte
  char sP_L = '}'; //send stop byte
  Serial1.putc(sT_L); Serial1.putc(sH_L); Serial1.putc(sL_L); Serial1.putc(sP_L);
}

void sendCmd_wheel_PWM_right()
{
  //cmd_wheel_pwm_right=100;
  int right_target_send = int(cmd_wheel_pwm_right/0.0077822); //convert rad/s to 16 bit integer to send 
  char sT_L = '{'; //send start byte
  uint8_t sH_L = highByte(right_target_send); //send high byte
  uint8_t sL_L = lowByte(right_target_send);  //send low byte
  char sP_L = '}'; //send stop byte
  Serial3.putc(sT_L); Serial3.putc(sH_L); Serial3.putc(sL_L); Serial3.putc(sP_L);
}





void readFeadback_angularVel_L()
{
    char buff[5];
    if(Serial1.readable() && Serial1.getc()=='{' ){
        Serial1.gets(buff,6);
        /*
                pc.putc(buff[0]);
                pc.putc(buff[1]);
                pc.putc(buff[2]);    
                pc.putc(buff[3]); 
                pc.putc(buff[4]); 
        */        
        if(buff[4]=='}'){
            int16_t left_actual_receive  = (buff[1] << 8) + buff[2];
            feedback_wheel_omega_left = left_actual_receive*double(MaxSpeed/32767);           
        } 
    }
}
void readFeadback_angularVel_R()
{
    char buff[5];
    if(Serial3.readable() && Serial3.getc()=='{' ){
        Serial3.gets(buff,6);
        /*
                pc.putc(buff[0]);
                pc.putc(buff[1]);
                pc.putc(buff[2]);    
                pc.putc(buff[3]); 
                pc.putc(buff[4]); 
        */        
        if(buff[4]=='}'){
            int16_t right_actual_receive  = (buff[1] << 8) + buff[2];
            feedback_wheel_omega_right = right_actual_receive*double(MaxSpeed/32767);           
        } 
    }
}

char hello[13] = "hello world!";

void timer1_Callback() {
    
        inverse_diff_model();
        model_PI_controller_update();

        //cmd_wheel_pwm_left = 30.0;
        //cmd_wheel_pwm_right = 30.0;      
          
        sendCmd_wheel_PWM_left();
        sendCmd_wheel_PWM_right();       
        
        cmd_wheel_angularVel_msg.x = cmd_wheel_omega_left;  
        cmd_wheel_angularVel_msg.y = cmd_wheel_omega_right;           
        cmd_wheel_angularVel_pub.publish( &cmd_wheel_angularVel_msg );  
        
        cmd_wheel_PWM_msg.x = cmd_wheel_pwm_left;  
        cmd_wheel_PWM_msg.y = cmd_wheel_pwm_right;             
        cmd_wheel_PWM_pub.publish( &cmd_wheel_PWM_msg );    



        readFeadback_angularVel_L();  
        readFeadback_angularVel_R();    
        feedback_wheel_angularVel_msg.x = feedback_wheel_omega_left;  
        feedback_wheel_angularVel_msg.y = feedback_wheel_omega_right;    
        feedback_wheel_angularVel_pub.publish( &feedback_wheel_angularVel_msg );   
 
        forward_diff_model();
        feedback_vel_msg.linear.x = feedback_model_vel_x;  
        feedback_vel_msg.angular.z = feedback_model_omega_z;    
        feedback_vel_pub.publish( &feedback_vel_msg );        
}

int main() {
    timer1.attach(&timer1_Callback, 0.1);    
    Serial1.baud(1000000); 
    Serial3.baud(1000000); 
    nh.initNode();
    nh.advertise(chatter);
    nh.advertise(feedback_vel_pub);
    nh.advertise(feedback_wheel_angularVel_pub);
    nh.advertise(cmd_wheel_angularVel_pub);    
    nh.advertise(cmd_wheel_PWM_pub);       
    nh.subscribe(cmd_vel_sub);   
    
    while (1) {
                        
        //Serial1.putc('S');                
        led = !led;
        
        nh.spinOnce();
        wait_ms(100);
    }
}

