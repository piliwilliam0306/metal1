
char commandArray[4];
byte sT = 0;  //send start byte
byte sA = 0;  //send device address + command byte
byte sH = 0;  //send high byte
byte sL = 0;  //send low byte
byte sP = 0;  //send stop byte

byte rT = 0;  //receive start byte
byte rA = 0;  //receive device address + command byte
byte rH = 0;  //receive high byte
byte rL = 0;  //receive low byte
byte rP = 0;  //receive stop byte

#define RS485Transmit    HIGH
#define RS485Receive     LOW 

//#define master 0
//#define slave 1
//#define slave 2
//#define slave 3
//#define slave 4

//#define slave 5
//#define slave 6
#define slave 7
//#define slave 8

unsigned long lastMilli = 0;                    // loop timing 
long dT = 0;

double omega_target = 0.0;
double omega_actual = 0;
int actual_send = 0;
int target_receive = 0;

void setup() { 
 pinMode(8, OUTPUT);
 digitalWrite(8, RS485Receive); //DE,RE=LOW, RX enabled
 Serial.begin (1000000);
} 

void loop() 
{       
  readCmd();
}

void readCmd()
{
  //digitalWrite(8, RS485Receive); //DE,RE=LOW, RX enabled
  if (Serial.available() >= 10) 
  {
    char rT = (char)Serial.read(); //read target speed from mega
          if(rT == '{')//start byte
            {
              char commandArray[9];
              Serial.readBytes(commandArray,9);
              byte rA=commandArray[0]; //device address + command
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
                  target_receive = (rH<<8)+rL; 
                  omega_target = double (target_receive*0.00958251953);  //convert received 16 bit integer to actual speed
                  sendFeedback();
                }
            }
  }         
}

void sendFeedback()
{
  digitalWrite(8, RS485Transmit); //DE,RE=HIGH, TX enabled
  //actual_send = int(omega_actual/0.00031434064); //convert rad/s to 16 bit integer to send
  actual_send = int(omega_target/0.00031434064); //convert rad/s to 16 bit integer to send
  char sT='{'; //send start byte
  byte sA = slave;//address + command
  byte sH = highByte(actual_send); //send high byte
  byte sL = lowByte(actual_send);  //send low byte
  char sH1 = 'R'; //any data you want to send
  char sL1 = 'O'; //any data you want to send
  char sH2 = 'B'; //any data you want to send
  char sL2 = 'O'; //any data you want to send
  char sH3 = 'T'; //any data you want to send
  char sP='}'; //send stop byte
  Serial.write(sT); Serial.write(sA); Serial.write(sH); Serial.write(sL); 
  Serial.write(sH1); Serial.write(sL1); Serial.write(sH2); Serial.write(sL2);
  Serial.write(sH3);
  Serial.write(sP);
  delayMicroseconds(10);
  digitalWrite(8, RS485Receive); //DE,RE=LOW, RX enabled
}

void printMotorInfo()  
{                                                                      
   Serial.print(" target:");                  Serial.print(omega_target);
   Serial.print(" actual:");                  Serial.print(omega_actual);
   Serial.println();
}

