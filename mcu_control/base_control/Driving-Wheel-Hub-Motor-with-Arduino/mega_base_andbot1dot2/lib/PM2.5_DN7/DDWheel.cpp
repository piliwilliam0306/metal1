#include "DDWheel.h"

#include "Arduino.h"

DDWheel::DDWheel(int pinA, int pinB, int pinC, int ind)
{
        Wheel_select = ind;
        encoderPinA = pinA;
        encoderPinB = pinB;
        encoderPinC = pinC;

	pinMode(encoderPinA, INPUT);
	pinMode(encoderPinB, INPUT);
	pinMode(encoderPinC, INPUT);
}

void DDWheel::Init()
{
        digitalWrite(encoderPinA, HIGH);
        digitalWrite(encoderPinB, HIGH);
        digitalWrite(encoderPinC, HIGH);

        pinAStateOld = digitalRead(encodePinA);
        pinBStateOld = digitalRead(encodePinB);
        pinCStateOld = digitalRead(encodePinC);

        attachInterrupt(0, doEncoder(), CHANGE);                                          //encoder pin on interrupt 0 - pin 2
        attachInterrupt(1, doEncoder(), CHANGE);                                          //encoder pin on interrupt 1 - pin 3
        attachInterrupt(2, doEncoder(), CHANGE);
}
void DDWheel::Enable()
{
    Encoderpos = 0;
    EncoderposPre = 0;
    //vol_target = 0;
    //sum_error = 0;
    Vq = 0;
    Vd = 0xFFFF;
    Serial.println("BLDC Enable");
    sendData[1] = highByte(Vq);
    sendData[2] = lowByte(Vq);
    sendData[3] = highByte(Vd);
    sendData[4] = lowByte(Vd);
    sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);
    Serial1.write(sendData, 7);
    Vd = 0x0;
}
void DDWheel::Disable()
{
    Encoderpos = 0;
    EncoderposPre = 0;
    //vol_target = 0;
    //sum_error = 0;
    Vq = 0;
    Vd = 0xAAAA;
    Serial.println("BLDC Disable");
    sendData[1] = highByte(Vq);
    sendData[2] = lowByte(Vq);
    sendData[3] = highByte(Vd);
    sendData[4] = lowByte(Vd);
    sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);
    Serial1.write(sendData, 7);
    Vd = 0x0;
}
void DDWheel::doEncoder()
{
    pinAState = digitalRead(encoderPinA);
    pinBState = digitalRead(encoderPinB);
    pinCState = digitalRead(encoderPinC);

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

void DDWheel::revFromMCU()
{
  if (Serial1.available()) {
      if (Serial1.read() == '{') {
        Serial1.readBytes(getData, 8);
        if (getData[7] == '}') {
          checksum = (0x55 ^ getData[0] ^ getData[1] ^ getData[2] ^ getData[3] ^ getData[4] ^ getData[5]);
          if (checksum == getData[6]) {
            for (int i = 0; i < 3; i ++) {
              MotorData[i].Byte.L = getData[2 * i];
              MotorData[i].Byte.H = getData[2 * i + 1];
            }
            Serial.println(String("MCU Vq=") + " " + \
                           String(MotorData[0].Data) + " " + \
                           String("MCU Vd=") + " " + \
                           String(MotorData[1].Data) + " " + \
                           String("MCU rds=") + " " + \
                           String(MotorData[2].Data));
          }
        }
      }
    }
}
void DDWheel::sendVoltCmd()
{
  if (Wheel_select == left)
  {
    Vq = -Vq;
  }
  else;



          if (Vq >= Vq_MAX)            Vq = Vq_MAX;
          else if (Vq <= Vq_MIN)    Vq = Vq_MIN;

//        if ((abs(Vq) <= VD_ENABLE_LIMITE) && (Vq != 0))
//        {
//              Vd = VD_ENABLE_VALUE;                                                         // The Vd always be postive value, even Vq is a negative value
//              digitalWrite(51, HIGH);                                                       // turn the LED on (HIGH is the voltage level)
//        }
//        else
//        {
//              Vd = 0;
//              digitalWrite(51, LOW);                                                        // turn the LED off by making the voltage LOW
//        }

          Vd = 0;
          sendData[1] = highByte(Vq);
          sendData[2] = lowByte(Vq);
          sendData[3] = highByte(Vd);
          sendData[4] = lowByte(Vd);
          sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);

          Serial1.write(sendData, 7);
}
void DDWheel::FbMotorData()
{
    EncodeDiff = Encoderpos - EncoderposPre;
    fb_omega = ((EncodeDiff) * (1000 / dT)) * 2 * PI / (CPR);

    if(Wheel_select == left)
    {
      fb_omega = -fb_omega;
    }
    else;


    //  Encoderpos=0;//recount
    EncoderposPre = Encoderpos;
//    Serial.println(String("EncodeDiff=") + " " + String(EncodeDiff));
//    Serial.println(String("dT=") + " " + String(dT));
}
//void DDWheel::Feedback_wheel_angularVel()
//{
//
//    if(WHEEL_SELECT==0) //left wheel
//      fb_omega= int(-1 * fb_omega / (MAX_AngularSpeed /double(32767)));           //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
//    else if(WHEEL_SELECT==1) //right wheel
//      fb_omega = int(fb_omega / (MAX_AngularSpeed /double(32767)));           //convert received 16 bit integer to actual speed 6.283/32767=1.917477950376904e-4=0.0001917477950376904
//
//  //  char sT = '{';                                                                  //send start byte
//  //  byte sH = highByte(actual_send);                                                //send high byte
//  //  byte sL = lowByte(actual_send);                                                 //send low byte
//  //  char sP = '}';                                                                  //send stop byte
//  //  Serial3.write(sT); Serial3.write(sH); Serial3.write(sL); Serial3.write(sP);
//}
