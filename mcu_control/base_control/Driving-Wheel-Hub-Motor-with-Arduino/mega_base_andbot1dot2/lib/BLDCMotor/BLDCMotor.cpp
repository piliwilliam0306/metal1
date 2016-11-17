/*
 * =====================================================================================
 *
 *       Filename:  BLDCMotor.cpp
 *
 *    Description:  This is a BLDC control library for differential drive mobile.
 *
 *        Version:  1.0
 *        Created:
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Weber-Hsu
 *   Organization:  Advanced Robotics Corporation
 *
 * =====================================================================================
 */
#include "BLDCMotor.h"

#include "Arduino.h"

BLDCMotor::BLDCMotor(SetENCPin* ENC_Pin, SetMotorParam* motor_param, SetCtrlParam* Ctrl_param, SetdqCmdLimit* Limit)
{
	Hall = *ENC_Pin;
	BLDCParam = *motor_param;
	BLDCCtrl = *Ctrl_param;
	InputLimit = *Limit;

	pinMode(Hall.pinA, INPUT);
	pinMode(Hall.pinB, INPUT);
	pinMode(Hall.pinC, INPUT);
}

void BLDCMotor::dqInputSelect(int qCmd, int dCmd)
{
	if (BLDCCtrl.mode == VqVdMode)
	{
		dqCmd.InputCmd[0] = qCmd;//dqCmd.Vq;
		dqCmd.InputCmd[1] = dCmd;//dqCmd.Vd;
	}
	else if (BLDCCtrl.mode == VqIdMode)
	{
		dqCmd.InputCmd[0] = qCmd;//dqCmd.Vq;
		dqCmd.InputCmd[1] = dCmd;//dqCmd.Vd;
	}
	else if (BLDCCtrl.mode == IqIdMode)
	{
		dqCmd.InputCmd[0] = qCmd;//dqCmd.Iq;
		dqCmd.InputCmd[1] = dCmd;;//dqCmd.Id;
	}
}
void BLDCMotor::Init()
{
	digitalWrite(Hall.pinA, HIGH);
	digitalWrite(Hall.pinB, HIGH);
	digitalWrite(Hall.pinC, HIGH);

	pinAStateOld = digitalRead(Hall.pinA);
	pinBStateOld = digitalRead(Hall.pinB);
	pinCStateOld = digitalRead(Hall.pinC);
}
void BLDCMotor::Enable()
{
    Encoderpos = 0;
    EncoderposPre = 0;
    dqInputSelect(0,0xFFFF); //Input q then d;
    //dqCmd.InputCmd[0] = 0; //Vq = 0;
    //dqCmd.InputCmd[1] = 0xFFFF; //Vd = 0xFFFF;
    Serial.println("BLDC Enable");
    sendData[1] = highByte(dqCmd.InputCmd[0]);
    sendData[2] = lowByte(dqCmd.InputCmd[0]);
    sendData[3] = highByte(dqCmd.InputCmd[1]);
    sendData[4] = lowByte(dqCmd.InputCmd[1]);
    sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);
    SerialSend2Driver(sendData,BLDCCtrl.axis);
//	if (BLDCCtrl.axis == left)	Serial2.write(sendData, 7);
//	else if (BLDCCtrl.axis == right) Serial3.write(sendData, 7);
	dqCmd.InputCmd[1] = 0x0;
}
void BLDCMotor::Disable()
{
    Encoderpos = 0;
    EncoderposPre = 0;
    dqInputSelect(0,0xAAAA); //Input q then d;
    //dqCmd.InputCmd[0] = 0;//Vq = 0;
    //dqCmd.InputCmd[1] = 0xAAAA;//Vd = 0xAAAA;
    Serial.println("BLDC Disable");
    sendData[1] = highByte(dqCmd.InputCmd[0]);
    sendData[2] = lowByte(dqCmd.InputCmd[0]);
    sendData[3] = highByte(dqCmd.InputCmd[1]);
    sendData[4] = lowByte(dqCmd.InputCmd[1]);
    sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);
    SerialSend2Driver(sendData,BLDCCtrl.axis);
//	if (BLDCCtrl.axis == left)	Serial2.write(sendData, 7);
//	else if (BLDCCtrl.axis == right) Serial3.write(sendData, 7);
	dqCmd.InputCmd[1] = 0x0;
}
void BLDCMotor::doEncoder()
{
    pinAState = digitalRead(Hall.pinA);
    pinBState = digitalRead(Hall.pinB);
    pinCState = digitalRead(Hall.pinC);

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

void BLDCMotor::revFromMCU()
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
void BLDCMotor::SendCmd()
{
	if (BLDCCtrl.axis == left) 		CmdRef.VoltCmd = -CmdRef.VoltCmd;
	else;
	dqInputSelect(CmdRef.VoltCmd*Vq_formatRatio,0); //Input q then d;Vq format transformation

	if (dqCmd.InputCmd[0] >= InputLimit.MaxVq)         dqCmd.InputCmd[0] = InputLimit.MaxVq;
	else if (dqCmd.InputCmd[0] <= InputLimit.MINVq)    dqCmd.InputCmd[0] = InputLimit.MINVq;

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
	sendData[1] = highByte(dqCmd.InputCmd[0]);
	sendData[2] = lowByte(dqCmd.InputCmd[0]);
	sendData[3] = highByte(dqCmd.InputCmd[1]);
	sendData[4] = lowByte(dqCmd.InputCmd[1]);
	sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);

	SerialSend2Driver(sendData,BLDCCtrl.axis);
//	if (BLDCCtrl.axis == left)	Serial2.write(sendData, 7);
//	else if (BLDCCtrl.axis == right) Serial3.write(sendData, 7);
}
void BLDCMotor::GetMotorData(long dT)
{
    EncodeDiff = Encoderpos - EncoderposPre;
    FbMotorInfo.AngularVel = ((EncodeDiff) * (1000 / dT)) * 2 * PI / (BLDCParam.CPR);

    if(BLDCCtrl.axis == left)
    {
    	FbMotorInfo.AngularVel = -FbMotorInfo.AngularVel;
    }
    else;

    //  Encoderpos=0;//recount
    EncoderposPre = Encoderpos;
//    Serial.println(String("EncodeDiff=") + " " + String(EncodeDiff));
//    Serial.println(String("dT=") + " " + String(dT));
}
void BLDCMotor::SerialSend2Driver(byte* sendData, int axis)
{
  if (axis == left)    Serial2.write(sendData, 7);
  else if (axis == right) Serial3.write(sendData, 7);
}
