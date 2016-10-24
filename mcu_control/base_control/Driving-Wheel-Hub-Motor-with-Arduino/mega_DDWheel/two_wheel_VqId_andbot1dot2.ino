/*
 * =====================================================================================
 *
 *       Filename:  two_wheel_MIMO_VqId_andbot1dot2.ino
 *
 *    Description:  The program is for wheel hub motor control (both left & right).
 *                  Vq max/min is 1000/-1000
 *                  Id max/min is 20/-20 A
 *
 *                  [HW Arduino Mega 2560]
 *                  Serial port (Default serial for debug )
 *                  Serial1 port (connect to Motor control board wheel)
 *                  Serial3 port (connect to the upper drive Serial1 port)
 *
 *                  [Control Board Communication Format]
 *                  Send Command Format: 7 Byte
 *                  Bytes:
 *                  Start  Vq_Hbyte Vq_Lbyte Vd_Hbyte Vd_Lbyte Checkcode  End
 *
 *                  ex:(Vq=300, Vd=4)
 *                  0x7B    0x01    0x2C      0x00    0x04      0x7C      0x7D
 *
 *                  ex:(Vq=0, Vd=0)
 *                  0x7B    0x00    0x00      0x00    0x00      0x55      0x7D
 *
 *                  Receive Data Format: 9 Byte
 *                  [Byte:1] [Byte:2]  [Byte:3]  [Byte:4]  [Byte:5]  [Byte:6]    [Byte:7]    [Byte:8]  [Byte:9]
 *                  Bytes:  Start    Vq_Hbyte  Vq_Lbyte  Vd_Hbyte  Vd_Lbyte  pulse/ms_H  pulse/ms_L  Checkcode  End
 *                  ex:    0x7B                                                                                 0x7D
 *
 *                  [ Encode value check ]
 *      Bit         LSB           MSB
 *                  A     B       C
 *      Pin         2     3       21 (Arduino Mega 2560 Board)
 *      WireColor   Blue  Green   Yellow
 *      ClockWise value:=1,3,2,6,4,5
 *      CountClockWise:=1,5,4,6,2,3
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

//#define WHEEL_SELECT 0 //for left wheel
#define WHEEL_SELECT 1 //for right wheel

#define LOOPTIME        100 //500 for test & draw
unsigned long lastMilli = 0;
long dT = 0;

// self-defined upper and lower limit of driver output (precaution)
#define Vq_MAX 1000 //520
#define Vq_MIN -1000 //-520
#define Id_MAX 20
#define Id_MIN -20
#define Id_formatRatio 100 //for raising resolution, according to student Tsai.

// data tramission format
union Data_Setting {
  struct _ByteSet {
    byte L;
    byte H;
  } Byte;
  int Data;
};

union Data_Setting MotorData[3];
byte getData[8];
byte sendData[7] = {123, 0, 0, 0, 0, 0, 125};
byte sendDataStop[7] = {123, 0, 0, 0, 0, 85, 125};

unsigned long PastTime = 0;
int Vq = 0, Id = 0, checksum = 0;

int encodePinA = 2;
int encodePinB = 3;
int encodePinC = 21;
volatile long Encoderpos = 0;
long EncoderposPre = 0;
volatile int lastEncoded = 0;

int pinAState = 0;
int pinAStateOld = 0;
int pinBState = 0;
int pinBStateOld = 0;
int pinCState = 0;
int pinCStateOld = 0;

double vol_target = 0;
double omega_actual = 0;

// parameters
int CPR = 90;                                                                     //encoder count per revolution
int gear_ratio = 1;
const double MAX_AngularSpeed = 450 / 60 * 2 * PI; // DD motor nominal rotation speed: 450 rpm

int actual_send = 0;
int target_receive = 0;

//test for encoder error
int EncodeDiff = 0;
int EncodeDiffPre = 0;

//restart condition for Warrning Encoder problem
int StartFlag = 0;
int receiveVq = 0;
byte receiveMode = 0;
byte actualMode = 0;

//Add Vd control
//int VD_ENABLE_LIMITE = 160;
//int VD_ENABLE_VALUE = 50;

void readCmd_wheel_volt()
{
    if (Serial3.available())
    {
      char rT = (char)Serial3.read();                                               //read target speed from mega

      if (rT == 'm')
      {                                                              //test by BT
          StartFlag = 1;
          Encoderpos = 0;
          EncoderposPre = 0;
          vol_target = 0;
          //sum_error = 0;
          Vq = 0;
          Id = 0xbbbb; //vd = 0xFFFF;
          Serial.println("BLDC Enable");
          sendData[1] = highByte(Vq);
          sendData[2] = lowByte(Vq);
          sendData[3] = highByte(Id);
          sendData[4] = lowByte(Id);
          sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);
          Serial1.write(sendData, 7);
          Id = 0x0;
      }
      else if (rT == 'k')
      {                                                              //test by BT
          StartFlag = 1;
          Encoderpos = 0;
          EncoderposPre = 0;
          vol_target = 0;
          //sum_error = 0;
          Vq = 0;
          Id = 0xaaaa;//vd = 0xAAAA;
          Serial.println("BLDC Disable");
          sendData[1] = highByte(Vq);
          sendData[2] = lowByte(Vq);
          sendData[3] = highByte(Id);
          sendData[4] = lowByte(Id);
          sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);
          Serial1.write(sendData, 7);
          Id = 0x0;
      }
      else if (rT == '{')
      {
          byte commandArray[3];
          Serial3.readBytes(commandArray, 3);
          byte rH = commandArray[0];
          byte rL = commandArray[1];
          char rP = commandArray[2];

          if (rP == '}')
          {
            target_receive = (rH << 8) + rL;
            vol_target = double (target_receive * double(Vq_MAX)/double(32767));          //convert received 16 bit integer to actual voltage => Vq_MAX/32767
            if(WHEEL_SELECT==0) //left wheel
            vol_target = -vol_target;
          }
       }
    }    //end of if (Serial3.available())
}

void sendFeedback_wheel_angularVel() {
  //limit 1 rev/sec
  if(WHEEL_SELECT==0) //left wheel
    actual_send = int(-1 * omega_actual / (MAX_AngularSpeed / double(32767)));           //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
  else if(WHEEL_SELECT==1) //right wheel
    actual_send = int(omega_actual / (MAX_AngularSpeed / double(32767)));           //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904

  char sT = '{';                                                                  //send start byte
  byte sH = highByte(actual_send);                                                //send high byte
  byte sL = lowByte(actual_send);                                                 //send low byte
  char sP = '}';                                                                  //send stop byte
  Serial3.write(sT); Serial3.write(sH); Serial3.write(sL); Serial3.write(sP);
}


void getMotorData()
{
  EncodeDiff = Encoderpos - EncoderposPre;
  omega_actual = ((EncodeDiff) * (1000 / dT)) * 2 * PI / (CPR);

  //  Encoderpos=0;//recount
  EncoderposPre = Encoderpos;
  Serial.println(String("EncodeDiff=") + " " + String(EncodeDiff));
  Serial.println(String("dT=") + " " + String(dT));
}

void sendCmd()
{
    if (Vq >= Vq_MAX)             Vq = Vq_MAX;
    else if (Vq <= Vq_MIN)        Vq = Vq_MIN;
    if (Id >= Id_MAX)             Id = Id_MAX;
    else if (Id <= Id_MIN)        Id = Id_MIN;

    sendData[1] = highByte(Vq);
    sendData[2] = lowByte(Vq);
    sendData[3] = highByte(Id);
    sendData[4] = lowByte(Id);
    sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);

    Serial1.write(sendData, 7);
}


void revFromMCU()
{
    if (Serial1.available())
    {
        if (Serial1.read() == '{')
        {
            Serial1.readBytes(getData, 8);
            if (getData[7] == '}')
            {
                checksum = (0x55 ^ getData[0] ^ getData[1] ^ getData[2] ^ getData[3] ^ getData[4] ^ getData[5]);
                if (checksum == getData[6])
                {
                    for (int i = 0; i < 3; i ++)
                    {
                      MotorData[i].Byte.L = getData[2 * i];
                      MotorData[i].Byte.H = getData[2 * i + 1];
                    }
                    Serial.println(String("MCU Vq=") + " " + \
                           String(MotorData[0].Data) + " " + \
                           String("MCU Id=") + " " + \
                           String(MotorData[1].Data) + " " + \
                           String("MCU rds=") + " " + \
                           String(MotorData[2].Data));
                }
            }
        }
    }
}


void doEncoder()
{
    pinAState = digitalRead(encodePinA);
    pinBState = digitalRead(encodePinB);
    pinCState = digitalRead(encodePinC);

    if (pinAState == 0 && pinBState == 1 && pinCState == 1) {                       //value:=1
      if (pinAStateOld == 0 && pinBStateOld == 1 && pinCStateOld == 0)              //value:=5 // CW
        Encoderpos ++;
      if (pinAStateOld == 0 && pinBStateOld == 0 && pinCStateOld == 1)              //value:=3 // CCW
        Encoderpos --;
    }

    if (pinAState == 0 && pinBState == 0 && pinCState == 1) {                       //value:=3
      if (pinAStateOld == 0 && pinBStateOld == 1 && pinCStateOld == 1)              //value:=1 // CW
        Encoderpos ++;
      if (pinAStateOld == 1 && pinBStateOld == 0 && pinCStateOld == 1)              //value:=2 // CCW
        Encoderpos --;
    }

    if (pinAState == 1 && pinBState == 0 && pinCState == 1) {                       //value:=2
      if (pinAStateOld == 0 && pinBStateOld == 0 && pinCStateOld == 1)              //value:=3 // CW
        Encoderpos ++;
      if (pinAStateOld == 1 && pinBStateOld == 0 && pinCStateOld == 0)              //value:=6 // CCW
        Encoderpos --;
    }

    if (pinAState == 1 && pinBState == 0 && pinCState == 0) {                       //value:=6
      if (pinAStateOld == 1 && pinBStateOld == 0 && pinCStateOld == 1)              //value:=2 // CW
        Encoderpos ++;
      if (pinAStateOld == 1 && pinBStateOld == 1 && pinCStateOld == 0)              //value:=4 // CCW
        Encoderpos --;
    }

    if (pinAState == 1 && pinBState == 1 && pinCState == 0) {                       //value:=4
      if (pinAStateOld == 1 && pinBStateOld == 0 && pinCStateOld == 0)              //value:=6 // CW
        Encoderpos ++;
      if (pinAStateOld == 0 && pinBStateOld == 1 && pinCStateOld == 0)              //value:=5 // CCW
        Encoderpos --;
    }

    if (pinAState == 0 && pinBState == 1 && pinCState == 0) {                       //value:=5
      if (pinAStateOld == 1 && pinBStateOld == 1 && pinCStateOld == 0)              //value:=4 // CW
        Encoderpos ++;
      if (pinAStateOld == 0 && pinBStateOld == 1 && pinCStateOld == 1)              //value:=1 // CCW
        Encoderpos --;
    }

    pinAStateOld = pinAState;
    pinBStateOld = pinBState;
    pinCStateOld = pinCState;
}

void setup()
{
    Serial3.begin(115200);                                                          //for upper device sned/receive command
    Serial1.begin(115200);                                                          //for commect to wheel control board
    Serial.begin(115200);                                                           //default serial port for debug

    pinMode(51, OUTPUT);                                                           //for VD_ENABLE_VALUE check

    pinMode(encodePinA, INPUT);
    pinMode(encodePinB, INPUT);
    pinMode(encodePinC, INPUT);
    digitalWrite(encodePinA, HIGH);                                                 //turn on pullup resistor
    digitalWrite(encodePinB, HIGH);                                                 //turn on pullup resistor
    digitalWrite(encodePinC, HIGH);                                                 //turn on pullup resistor

    pinAStateOld = digitalRead(encodePinA);
    pinBStateOld = digitalRead(encodePinB);
    pinCStateOld = digitalRead(encodePinC);

    attachInterrupt(0, doEncoder, CHANGE);                                          //encoder pin on interrupt 0 - pin 2
    attachInterrupt(1, doEncoder, CHANGE);                                          //encoder pin on interrupt 1 - pin 3
    attachInterrupt(2, doEncoder, CHANGE);                                          //encoder pin on interrupt 2 - pin 21
}


void loop()
{
    readCmd_wheel_volt();                 //include the initial command

    if ((millis() - lastMilli) >= LOOPTIME)
    {
        dT = millis() - lastMilli;
        lastMilli = millis();

        getMotorData();
        sendFeedback_wheel_angularVel();    //send actually speed to mega

        if (vol_target == 0)
        {
            Vq = 0;
            //sum_error = 0;
        }
        else
        {
            Vq = vol_target;
        }

        Id = 0.0 * Id_formatRatio;

        if (Vq == 0)
            Serial.println("Current Vq is 0");

        if (Id == 0)
            Serial.println("Current Id is 0");
        sendCmd();
    }
}
