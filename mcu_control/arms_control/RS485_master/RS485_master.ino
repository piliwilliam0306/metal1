
#include <ros.h>
#include <geometry_msgs/Vector3.h>
#define LOOPTIME 100

#define RS485Transmit    HIGH
#define RS485Receive     LOW 

//#define slave 0
#define slave1 1
#define slave2 2
#define slave3 3
#define slave4 4
#define slave5 5
#define slave6 6
#define slave7 7
#define slave8 8
#define slave9 9
#define slave10 10

unsigned long lastMilli = 0;                    // loop timing 
long dT = 0;
unsigned long cc = 0;

double rad_tick=(double)62.83/32767; //10rev 2^15
double degree_tick=(double)3600/32767; //

double theta_target1 = 0.0;
double theta_target2 = 0.0;
double theta_target3 = 0.0;
double theta_target4 = 0.0;
double theta_target5 = 0.0;
double theta_target6 = 0.0;
double theta_target7 = 0.0;
double theta_target8 = 0.0;

ros::NodeHandle nh;

bool set_; 

geometry_msgs::Vector3 vel_msg1;
ros::Publisher p1("feedback1", &vel_msg1);

geometry_msgs::Vector3 vel_msg2;
ros::Publisher p2("feedback2", &vel_msg2);

geometry_msgs::Vector3 vel_msg3;
ros::Publisher p3("feedback3", &vel_msg3);

geometry_msgs::Vector3 vel_msg4;
ros::Publisher p4("feedback4", &vel_msg4);

geometry_msgs::Vector3 vel_msg5;
ros::Publisher p5("feedback5", &vel_msg5);

geometry_msgs::Vector3 vel_msg6;
ros::Publisher p6("feedback6", &vel_msg6);

geometry_msgs::Vector3 vel_msg7;
ros::Publisher p7("feedback7", &vel_msg7);

geometry_msgs::Vector3 vel_msg8;
ros::Publisher p8("feedback8", &vel_msg8);

void messageCb1(const geometry_msgs::Vector3& msg1)
{
  theta_target1 = msg1.x;  
  //theta_target1 = msg1.y;
}

void messageCb2(const geometry_msgs::Vector3& msg2)
{
  theta_target2 = msg2.x;  
  //theta_target2 = msg2.y;
}

void messageCb3(const geometry_msgs::Vector3& msg3)
{
  theta_target3 = msg3.x;  
  //theta_target3 = msg3.y;
}

void messageCb4(const geometry_msgs::Vector3& msg4)
{
  theta_target4 = msg4.x;  
  //theta_target4 = msg4.y;
}

void messageCb5(const geometry_msgs::Vector3& msg5)
{
  theta_target5 = msg5.x;  
  //theta_target1 = msg1.y;
}

void messageCb6(const geometry_msgs::Vector3& msg6)
{
  theta_target6 = msg6.x;  
  //theta_target2 = msg2.y;
}

void messageCb7(const geometry_msgs::Vector3& msg7)
{
  theta_target7 = msg7.x;  
  //theta_target3 = msg3.y;
}

void messageCb8(const geometry_msgs::Vector3& msg8)
{
  theta_target8 = msg8.x;  
  //theta_target4 = msg4.y;
}

ros::Subscriber<geometry_msgs::Vector3> s1("cmd1",messageCb1);
ros::Subscriber<geometry_msgs::Vector3> s2("cmd2",messageCb2);
ros::Subscriber<geometry_msgs::Vector3> s3("cmd3",messageCb3);
ros::Subscriber<geometry_msgs::Vector3> s4("cmd4",messageCb4);
ros::Subscriber<geometry_msgs::Vector3> s5("cmd5",messageCb5);
ros::Subscriber<geometry_msgs::Vector3> s6("cmd6",messageCb6);
ros::Subscriber<geometry_msgs::Vector3> s7("cmd7",messageCb7);
ros::Subscriber<geometry_msgs::Vector3> s8("cmd8",messageCb8);

void setup() { 
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(s1);
  nh.subscribe(s2);
  nh.subscribe(s3);
  nh.subscribe(s4);
  nh.advertise(p1);  
  nh.advertise(p2);  
  nh.advertise(p3);  
  nh.advertise(p4);  
  nh.subscribe(s5);
  nh.subscribe(s6);
  nh.subscribe(s7);
  nh.subscribe(s8);
  nh.advertise(p5);  
  nh.advertise(p6);  
  nh.advertise(p7);  
  nh.advertise(p8); 
  pinMode(8, OUTPUT);
  digitalWrite(8, RS485Receive); //DE,RE=LOW, RX enabled
  Serial1.begin (1000000);
  //Serial.begin (115200);
} 

void loop() 
{       
  if((millis()-lastMilli) >= LOOPTIME)   
                    {                                    // enter tmed loop
                        dT = millis()-lastMilli;
                        lastMilli = millis();
//                        delay(1);
                        sendCmd(theta_target1, slave1);
//                        
//                        vel_msg1.x = read_slave(slave1);
//                        p1.publish(&vel_msg1);
//                        delay(1);
                        sendCmd(theta_target2, slave2);
//                        
//                        vel_msg2.x = read_slave(slave2);
//                        p2.publish(&vel_msg2);
//                        delay(1);
                        sendCmd(theta_target3, slave3);
//                        
//                        vel_msg3.x = read_slave(slave3);
//                        p3.publish(&vel_msg3);
//                        delay(1);
                        sendCmd(theta_target4, slave4);
//                        
//                        vel_msg4.x = read_slave(slave4);
//                        p4.publish(&vel_msg4);
//                        delay(1);
//                        sendCmd(theta_target5, slave5);
//                        
//                        vel_msg5.x = read_slave(slave5);
//                        p5.publish(&vel_msg5);
//                        delay(1);
//                        sendCmd(theta_target6, slave6);
//                        
//                        vel_msg6.x = read_slave(slave6);
//                        p6.publish(&vel_msg6);
//                        delay(1);
//                        sendCmd(theta_target7, slave7);
                        
//                        vel_msg7.x = read_slave(slave7);
//                        p7.publish(&vel_msg7);
//                        delay(1);
//                        sendCmd(theta_target8, slave8);
                        
//                        vel_msg8.x = read_slave(slave8);
//                        p8.publish(&vel_msg8);
                        
                        //printMotorInfo();
                    }
  nh.spinOnce();
}

double read_slave(int slave)
{
  double omega_actual=0;;
  int actual_receive=0;;
  /*byte rT = 0;  //receive start byte
  byte rA = 0;  //receive device address + command byte
  byte rH = 0;  //receive high byte
  byte rL = 0;  //receive low byte
  byte rH1 = 0;  //receive high byte
  byte rL1 = 0;  //receive low byte
  byte rP = 0;  //receive stop byte*/
  if (Serial1.available() >= 10) 
  {
    char rT = (char)Serial1.read(); //read target speed from mega
          if(rT == '{')//start byte
            {
              char commandArray[9];
              Serial1.readBytes(commandArray,9);
              byte rA=commandArray[0]; //device address
              byte rH=commandArray[1]; //high byte
              byte rL=commandArray[2]; //low byte
              byte rH1=commandArray[3]; //high byte
              byte rL1=commandArray[4]; //low byte
              byte rH2=commandArray[5]; //high byte
              byte rL2=commandArray[6]; //low byte
              byte rH3=commandArray[7]; //high byte
              char rP=commandArray[8]; //stop byte
              if(rP=='}' && rA==slave)         
                {
                  actual_receive = (rH<<8)+rL; 
                  omega_actual = double (actual_receive*0.00031434064);  //convert received 16 bit integer to actual speed
                  return omega_actual;
                }
            }
  }         
}

void sendCmd(double theta_target,int slave)
{
  int target_send;
  digitalWrite(8, RS485Transmit); //DE,RE=HIGH, TX enabled
  target_send = int(theta_target/rad_tick); //convert rad/s to 16 bit integer to send  100*6.28/2^16
  char sT='{';  //send start byte
  byte sA = slave;
  byte sH = highByte(target_send); //send high byte
  byte sL = lowByte(target_send);  //send low byte
  char sH1 = 'R'; //send high byte
  char sL1 = 'O';  //send low byte
  char sH2 = 'B'; //send high byte
  char sL2 = 'O';  //send low byte
  char sH3 = 'T'; //send high byte
  char sP='}';  //send stop byte

  Serial1.write(sT); 
  Serial1.write(sA); 
  Serial1.write(sH); Serial1.write(sL); 
  Serial1.write(sH1); Serial1.write(sL1); 
  Serial1.write(sH2); Serial1.write(sL2);
  Serial1.write(sH3);  
  Serial1.write(sP); 
  delayMicroseconds(10);
  digitalWrite(8, RS485Receive); //DE,RE=LOW, RX enabled
  //delay(1);
}

void printMotorInfo()  
{                                                                      
   Serial.print(" target1:");                  Serial.print(theta_target1);
   //Serial.print(" actual1:");                  Serial.print(omega_actual1);
   //Serial.print(" actual:");                  Serial.print(omega_actual);
   Serial.println();
   /*Serial.print(" target2:");                  Serial.print(theta_target2);
   Serial.print(" actual2:");                  Serial.print(omega_actual2);
   Serial.println();*/
}

