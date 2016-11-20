//#define USE_USBCON
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#define MaxSpeed 10.96
#define MaxPWM 255
int MaxSumError_L = 24;
int MaxSumError_W = 24;

//function prototype start
void cmd_velCallback(const geometry_msgs::Twist& cmdVel_msg);
void readFeadback_angularVel_L();
void model_PI_controller_update();
void sendCmd_wheel_PWM_L();
void sendCmd_wheel_PWM_R();
void inverse_diff_model();
void forward_diff_model();
void param_L_gainCallback(const geometry_msgs::Vector3& param_L_gain_msg);
void param_W_gainCallback(const geometry_msgs::Vector3& param_W_gain_msg);
void OpenLoop_Switch_Callback(const std_msgs::UInt8& OpenLoop_Switch_msg);
void Calculate_FeedForward_term();



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
#define LOOPTIME        1
#define TOPIC_LOOPTIME        10
//global variable Start

//Control Stratergy
int OpenLoop_Switch = 0;

//Mechanical Param
double wheelSeparation = 0.405;
double wheelRadius = 0.078;

//cmd
double cmd_model_vel_x=0.0;
double cmd_model_omega_z=0.0;
double cmd_wheel_omega_left = 0.0;
double cmd_wheel_omega_right = 0.0;

//mimo model
//double FeedForward_Vol = 3.7;
//double FeedForward_Vol = 2.4;//outdoor
double FeedForward_Vol = 3.2;
double BackEMF_Compensate_Vol_linear_Constant = 18;
double BackEMF_Compensate_Vol_angular_Constant = 4;

double Kp_L = 1.0;
double Ki_L = 0.1;
double Kp_W = 4.0;
double Ki_W = 0.07;

double us_c = 0;
double ud_c = 0;
double us_ff = 0;
double ud_ff = 0;
double us = 0;
double ud = 0;

double uL_c = 0;
double uR_c = 0;
double uL_ff = 0;
double uR_ff = 0;
double uL = 0;
double uR = 0;

double sum_error_L = 0;
double sum_error_W = 0;

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
unsigned long lastFeedback = 0;

long dT = 0;

//ROS init Start
ros::NodeHandle nh;
geometry_msgs::Twist feedback_vel_msg;
geometry_msgs::Vector3 feedback_wheel_angularVel_msg;
geometry_msgs::Vector3 cmd_wheel_angularVel_msg;
geometry_msgs::Vector3 cmd_wheel_PWM_msg;
geometry_msgs::Vector3 cmd_us_ud_msg;
geometry_msgs::Vector3 param_L_gain_msg;
geometry_msgs::Vector3 param_W_gain_msg;
geometry_msgs::Vector3 cmd_vel_mobile_msg;

std_msgs::UInt8 OpenLoop_Switch_msg;
geometry_msgs::Vector3 FeedForward_Vol_msg;

void FeedForward_Vol_Callback(const geometry_msgs::Vector3& FeedForward_Vol_msg);
ros::Subscriber<std_msgs::UInt8> OpenLoop_Switch_sub("/andbot/OpenLoop_Switch", OpenLoop_Switch_Callback);
ros::Subscriber<geometry_msgs::Vector3> param_L_gain_sub("/andbot/gain_L", param_L_gainCallback);
ros::Subscriber<geometry_msgs::Vector3> param_W_gain_sub("/andbot/gain_W", param_W_gainCallback);
ros::Subscriber<geometry_msgs::Vector3> FeedForward_Vol_sub("/andbot/FeedForward_Vol", FeedForward_Vol_Callback);

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/andbot/cmd_vel", cmd_velCallback);
ros::Publisher cmd_wheel_angularVel_pub("/andbot/cmd_wheel_angularVel", &cmd_wheel_angularVel_msg);
ros::Publisher cmd_wheel_PWM_pub("/andbot/cmd_wheel_PWM", &cmd_wheel_PWM_msg);
ros::Publisher feedback_wheel_angularVel_pub("/andbot/feedback_wheel_angularVel", &feedback_wheel_angularVel_msg);
ros::Publisher feedback_vel_pub("/andbot/feedback_vel", &feedback_vel_msg);
ros::Publisher cmd_us_ud_pub("/andbot/cmd_us_ud", &cmd_us_ud_msg);
ros::Publisher cmd_vel_mobile_pub("/andbot/cmd_vel_mobile_mega", &cmd_vel_mobile_msg);


//ROS init END

void FeedForward_Vol_Callback(const geometry_msgs::Vector3& FeedForward_Vol_msg){
	FeedForward_Vol = FeedForward_Vol_msg.x;
	BackEMF_Compensate_Vol_linear_Constant = FeedForward_Vol_msg.y;
	BackEMF_Compensate_Vol_angular_Constant = FeedForward_Vol_msg.z;

}

void OpenLoop_Switch_Callback(const std_msgs::UInt8& OpenLoop_Switch_msg){
	OpenLoop_Switch=OpenLoop_Switch_msg.data;
}

void Calculate_FeedForward_term(){

	if (cmd_wheel_omega_left > 0){
		uL_ff = FeedForward_Vol;
		}
	else if (cmd_wheel_omega_left < 0)
		uL_ff = -FeedForward_Vol;
		else uL_ff = 0;
	uL = uL_c + uL_ff;

	if (cmd_wheel_omega_right > 0){
		uR_ff = FeedForward_Vol;
		}
	else if (cmd_wheel_omega_right < 0)
		uR_ff = -FeedForward_Vol;
		else uR_ff = 0;
	uR = uR_c + uR_ff;
}

void param_L_gainCallback(const geometry_msgs::Vector3& param_L_gain_msg)
{  
    Kp_L = param_L_gain_msg.x;
    Ki_L = param_L_gain_msg.y;
}

void param_W_gainCallback(const geometry_msgs::Vector3& param_W_gain_msg)
{  
    Kp_W = param_W_gain_msg.x;
    Ki_W = param_W_gain_msg.y;
}

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
/*
double updatePid(double targetValue,double currentValue,double Kp,double Ki,double &sum_error,int MaxSumError)   
{              
  double error = targetValue - currentValue; 
  sum_error = sum_error + error * dT *0.001;
  sum_error = constrain(sum_error, -MaxSumError, MaxSumError);
  double pidTerm = Kp * error + Ki * sum_error;   

  return pidTerm;
} 
*/
double updatePid(double targetValue,double currentValue,double Kp,double Ki,double &sum_error,int MaxSumError)   
{              
  double error = targetValue - currentValue; 
  sum_error = sum_error + error * dT *0.001;
  double iTerm = Ki * sum_error;
  iTerm = constrain(iTerm, -MaxSumError, MaxSumError);
  double pidTerm = Kp * error + iTerm;   

  return pidTerm;
} 


double BackEMF_Compensate_Vol_linear(double targetValue)   
{              
  double Backemf_Term = targetValue*BackEMF_Compensate_Vol_linear_Constant;   

  return Backemf_Term;
} 
double BackEMF_Compensate_Vol_angular(double targetValue)   
{              
  double Backemf_Term = targetValue*BackEMF_Compensate_Vol_angular_Constant;   

  return Backemf_Term;
} 

void model_PI_controller_update(){

    us = updatePid(cmd_model_vel_x, feedback_model_vel_x,Kp_L,Ki_L,sum_error_L,MaxSumError_L)+BackEMF_Compensate_Vol_linear(cmd_model_vel_x);
    ud = updatePid(cmd_model_omega_z, feedback_model_omega_z,Kp_W,Ki_W,sum_error_W,MaxSumError_W)+BackEMF_Compensate_Vol_angular(cmd_model_omega_z);

    uL_c = 0.5 * (us-ud);  
	uR_c = 0.5 * (us+ud);

	Calculate_FeedForward_term();

    cmd_wheel_pwm_left = uL*double(255/12);
    if (cmd_wheel_pwm_left>=255)
        cmd_wheel_pwm_left=255;
    else if (cmd_wheel_pwm_left<=-255)
        cmd_wheel_pwm_left=-255;

    cmd_wheel_pwm_right = uR*double(255/12);
    if (cmd_wheel_pwm_right>=255)
        cmd_wheel_pwm_right=255;
    else if (cmd_wheel_pwm_right<=-255)
        cmd_wheel_pwm_right=-255;
          
}
void model_OpenLoop_update(){

    us = (cmd_model_vel_x);
    ud = (cmd_model_omega_z);

    uL_c = 0.5 * (us-ud);  
	uR_c = 0.5 * (us+ud);

	Calculate_FeedForward_term();

    cmd_wheel_pwm_left = uL*double(255/12);
    if (cmd_wheel_pwm_left>=255)
        cmd_wheel_pwm_left=255;
    else if (cmd_wheel_pwm_left<=-255)
        cmd_wheel_pwm_left=-255;

    cmd_wheel_pwm_right = uR*double(255/12);
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
  nh.advertise(cmd_us_ud_pub);       
  nh.advertise(cmd_vel_mobile_pub);     
  nh.subscribe(cmd_vel_sub);   
  nh.subscribe(param_L_gain_sub);
  nh.subscribe(param_W_gain_sub);
  nh.subscribe(OpenLoop_Switch_sub);
  nh.subscribe(FeedForward_Vol_sub);

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

		  if (OpenLoop_Switch == 0){
	      	model_PI_controller_update();
			}
		  else
		  	model_OpenLoop_update();

       	  //cmd_wheel_pwm_left = 30.0;
          //cmd_wheel_pwm_right = 30.0;      
          
    	  sendCmd_wheel_PWM_L();
	      sendCmd_wheel_PWM_R();       
	}
   if((millis()-lastFeedback) >= TOPIC_LOOPTIME)   
{
          lastFeedback = millis();
          cmd_wheel_angularVel_msg.x = cmd_wheel_omega_left;  
    	  cmd_wheel_angularVel_msg.y = cmd_wheel_omega_right;           
	      cmd_wheel_angularVel_pub.publish( &cmd_wheel_angularVel_msg );  

          cmd_us_ud_msg.x = us;  
    	  cmd_us_ud_msg.y = ud;             
	      cmd_us_ud_pub.publish( &cmd_us_ud_msg );    


        
          cmd_wheel_PWM_msg.x = cmd_wheel_pwm_left;  
    	  cmd_wheel_PWM_msg.y = cmd_wheel_pwm_right;             
	      cmd_wheel_PWM_pub.publish( &cmd_wheel_PWM_msg );    

          cmd_vel_mobile_msg.x = cmd_model_vel_x;  
    	  cmd_vel_mobile_msg.z = cmd_model_omega_z;             
	      cmd_vel_mobile_pub.publish( &cmd_vel_mobile_msg );    


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




