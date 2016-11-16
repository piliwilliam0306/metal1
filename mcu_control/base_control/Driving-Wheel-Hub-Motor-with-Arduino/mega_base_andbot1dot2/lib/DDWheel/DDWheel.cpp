#include "DDWheel.h"

#include "Arduino.h"

DDWheel::DDWheel(SetENCPin* ENC_Pin, SetMotorParam* motor_param, SetCtrlParam* Ctrl_param, SetdqCmdLimit* Limit)
{
	Hall = *ENC_Pin;
	BLDC_DDWheel = *motor_param;
	DDWheel_Ctrl = *Ctrl_param;
	InputLimit = *Limit;

	if (DDWheel_Ctrl.mode == VqVdMode)
	{
		InputCmd[0] = Vq;
		InputCmd[1] = Vd;
	}
	else if (DDWheel_Ctrl.mode == VqIdMode)
	{
		InputCmd[0] = Vq;
		InputCmd[1] = Vd;
	}
	else if (DDWheel_Ctrl.mode == IdIqMode)
	{
		InputCmd[0] = Id;
		InputCmd[1] = Iq;
	}

	pinMode(Hall.pinA, INPUT);
	pinMode(Hall.pinB, INPUT);
	pinMode(Hall.pinC, INPUT);
}

void DDWheel::Init()
{
	digitalWrite(Hall.pinA, HIGH);
	digitalWrite(Hall.pinB, HIGH);
	digitalWrite(Hall.pinC, HIGH);

	pinAStateOld = digitalRead(Hall.pinA);
	pinBStateOld = digitalRead(Hall.pinB);
	pinCStateOld = digitalRead(Hall.pinC);
}
void DDWheel::Enable()
{
    Encoderpos = 0;
    EncoderposPre = 0;
    //vol_target = 0;
    //sum_error = 0;
    InputCmd[0] = 0; //Vq = 0;
    InputCmd[1] = 0xFFFF; //Vd = 0xFFFF;
    Serial.println("BLDC Enable");
    sendData[1] = highByte(InputCmd[0]);
    sendData[2] = lowByte(InputCmd[0]);
    sendData[3] = highByte(InputCmd[1]);
    sendData[4] = lowByte(InputCmd[1]);
    sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);
    Serial1.write(sendData, 7);
    InputCmd[1] = 0x0;
}
void DDWheel::Disable()
{
    Encoderpos = 0;
    EncoderposPre = 0;
    //vol_target = 0;
    //sum_error = 0;
    InputCmd[0] = 0;//Vq = 0;
    InputCmd[1] = 0xAAAA;//Vd = 0xAAAA;
    Serial.println("BLDC Disable");
    sendData[1] = highByte(InputCmd[0]);
    sendData[2] = lowByte(InputCmd[0]);
    sendData[3] = highByte(InputCmd[1]);
    sendData[4] = lowByte(InputCmd[1]);
    sendData[5] = (0x55 ^ sendData[1] ^ sendData[2] ^ sendData[3] ^ sendData[4]);
    Serial1.write(sendData, 7);
    InputCmd[1] = 0x0;
}
void DDWheel::doEncoder()
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
	if (DDWheel_Ctrl.axis == left) 		cmd_volt = -cmd_volt;
	else;

	Vq = cmd_volt * Vq_formatRatio;

	if (Vq >= InputLimit.MaxVq)         Vq = InputLimit.MaxVq;
	else if (Vq <= InputLimit.MINVq)    Vq = InputLimit.MINVq;

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

	if (DDWheel_Ctrl.axis == left)	Serial1.write(sendData, 7);
	else if (DDWheel_Ctrl.axis == right) Serial2.write(sendData, 7);
}
void DDWheel::FbMotorData(long dT)
{
    EncodeDiff = Encoderpos - EncoderposPre;
    fb_omega = ((EncodeDiff) * (1000 / dT)) * 2 * PI / (BLDC_DDWheel.CPR);

    if(DDWheel_Ctrl.axis == left)
    {
      fb_omega = -fb_omega;
    }
    else;

    //  Encoderpos=0;//recount
    EncoderposPre = Encoderpos;
//    Serial.println(String("EncodeDiff=") + " " + String(EncodeDiff));
//    Serial.println(String("dT=") + " " + String(dT));
}
