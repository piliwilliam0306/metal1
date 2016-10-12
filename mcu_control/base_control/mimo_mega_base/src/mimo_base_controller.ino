//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define MaxSpeed 10.96
#define MaxPWM 255

//function prototype start
void cmd_velCallback(const geometry_msgs::Twist& cmdVel_msg);
void readFeadback_angularVel_L();
void model_PI_controller_update();
void sendCmd_wheel_PWM_L();
void sendCmd_wheel_PWM_R();
void inverse_diff_model();
void forward_diff_model();
//function prototype End

//////////////////////////////
//Hardware Configuration Start
//////////////////////////////

//only the following can be used for RX: 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
/*
const int Tx_R = 13;
const int Rx_R = 12;

const int Tx_L = 11;
const int Rx_L = 10; 
*/
#define LOOPTIME        40
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

unsigned long lastMilli = 0;
long dT = 0;

//ROS init Start
ros::NodeHandle nh;
geometry_msgs::Twist feedback_vel_msg;
geometry_msgs::Vector3 feedback_wheel_angularVel_msg;
geometry_msgs::Vector3 cmd_wheel_angularVel_msg;
geometry_msgs::Vector3 cmd_wheel_PWM_msg;

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/andbot/cmd_vel", cmd_velCallback);
ros::Publisher cmd_wheel_angularVel_pub("/andbot/cmd_wheel_angularVel", &cmd_wheel_angularVel_msg);
ros::Publisher cmd_wheel_PWM_pub("/andbot/cmd_wheel_PWM", &cmd_wheel_PWM_msg);
ros::Publisher feedback_wheel_angularVel_pub("/andbot/feedback_wheel_angularVel", &feedback_wheel_angularVel_msg);
ros::Publisher feedback_vel_pub("/andbot/feedback_vel", &feedback_vel_msg);

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

void sendCmd_wheel_PWM_L()
{
  //cmd_wheel_pwm_left=100;
  int left_target_send = int(cmd_wheel_pwm_left/0.0077822); //convert rad/s to 16 bit integer to send 
  char sT_L = '{'; //send start byte
  byte sH_L = highByte(left_target_send); //send high byte
  byte sL_L = lowByte(left_target_send);  //send low byte
  char sP_L = '}'; //send stop byte
  Serial1.write(sT_L); Serial1.write(sH_L); Serial1.write(sL_L); Serial1.write(sP_L);
}

void sendCmd_wheel_PWM_R()
{
  //cmd_wheel_pwm_right=100;
  int right_target_send = int(cmd_wheel_pwm_right/0.0077822); //convert rad/s to 16 bit integer to send 
  char sT_L = '{'; //send start byte
  byte sH_L = highByte(right_target_send); //send high byte
  byte sL_L = lowByte(right_target_send);  //send low byte
  char sP_L = '}'; //send stop byte
  Serial2.write(sT_L); Serial2.write(sH_L); Serial2.write(sL_L); Serial2.write(sP_L);
}





void readFeadback_angularVel_L()
{
    char rT_L = (char)Serial1.read(); //read actual speed from Uno
    if(rT_L == '{')
      {
        char commandArray_L[3];
        Serial1.readBytes(commandArray_L,4);
        byte rH_L = commandArray_L[0];
        byte rL_L = commandArray_L[1];
        char rP_L = commandArray_L[3];
        if(rP_L=='}'){
            int left_actual_receive  = (rH_L << 8) + rL_L;
            feedback_wheel_omega_left = left_actual_receive*double(MaxSpeed/32767);           
        } 
    }
}

void readFeadback_angularVel_R()
{
    char rT_L = (char)Serial2.read(); //read actual speed from Uno
    if(rT_L == '{')
      {
        char commandArray_R[3];
        Serial2.readBytes(commandArray_R,4);
        byte rH_R = commandArray_R[0];
        byte rL_R = commandArray_R[1];
        char rP_R = commandArray_R[3];
        if(rP_R=='}'){
            int right_actual_receive  = (rH_R << 8) + rL_R;
            feedback_wheel_omega_right = right_actual_receive*double(MaxSpeed/32767);           
        } 
    }
}

void setup() 
{
  nh.getHardware()->setBaud(1000000);
  nh.initNode();
  nh.advertise(feedback_vel_pub);
  nh.advertise(feedback_wheel_angularVel_pub);
  nh.advertise(cmd_wheel_angularVel_pub);    
  nh.advertise(cmd_wheel_PWM_pub);       
  nh.subscribe(cmd_vel_sub);   
  //The Arduino Mega has three additional serial ports: 
  //Serial1 on pins 19 (RX) and 18 (TX), 
  //Serial2 on pins 17 (RX) and 16 (TX), 
  //Serial on pins 15 (RX) and 14 (TX). 
  Serial2.begin (1000000);  //right
  Serial1.begin (1000000);  //left
  Serial.begin (1000000);  //left

}

void loop() 
{
  readFeadback_angularVel_L();
  readFeadback_angularVel_R();   
  forward_diff_model();
  if((millis()-lastMilli) >= LOOPTIME)   
       {                                    // enter tmed loop
          dT = millis()-lastMilli;
          lastMilli = millis();

    	  inverse_diff_model();
	      model_PI_controller_update();

       	  //cmd_wheel_pwm_left = 30.0;
          //cmd_wheel_pwm_right = 30.0;      
          
    	  sendCmd_wheel_PWM_L();
	      sendCmd_wheel_PWM_R();       
        
          cmd_wheel_angularVel_msg.x = cmd_wheel_omega_left;  
    	  cmd_wheel_angularVel_msg.y = cmd_wheel_omega_right;           
	      cmd_wheel_angularVel_pub.publish( &cmd_wheel_angularVel_msg );  
        
          cmd_wheel_PWM_msg.x = cmd_wheel_pwm_left;  
    	  cmd_wheel_PWM_msg.y = cmd_wheel_pwm_right;             
	      cmd_wheel_PWM_pub.publish( &cmd_wheel_PWM_msg );    


          feedback_wheel_angularVel_msg.x = feedback_wheel_omega_left;  
    	  feedback_wheel_angularVel_msg.y = feedback_wheel_omega_right;    
	      feedback_wheel_angularVel_pub.publish( &feedback_wheel_angularVel_msg );   
 

       	  feedback_vel_msg.linear.x = feedback_model_vel_x;  
    	  feedback_vel_msg.angular.z = feedback_model_omega_z;    
	      feedback_vel_pub.publish( &feedback_vel_msg );        
          //printMotorInfo();
       }     
	nh.spinOnce();      
}


void printMotorInfo()  
{                                                                      
   Serial.print("  LEFT target rad/s:");                  Serial.print(cmd_wheel_pwm_left);

   Serial.print("  RIGHT target rad/s:");                  Serial.print(cmd_wheel_pwm_right);
   Serial.println(); 
   Serial.println(); 

}




