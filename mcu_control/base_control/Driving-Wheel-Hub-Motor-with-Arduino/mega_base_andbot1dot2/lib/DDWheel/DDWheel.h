#ifndef DDWheel_h
#define DDWheel_h

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#endif

class DDWheel {
	public:
		// motor parameters
		typedef struct{
			int CPR;
			int gear_ratio;
			double MAX_AngularSpeed;
		}SetMotorParam;

		struct SetENCPin{
			int pinA;
			int pinB;
			double pinC;
		};

		struct SetCtrlParam{
			int axis;
			int mode;
		};

		typedef struct{
			int MAXVoltCmd;
			int MINVoltCmd;
			int MaxVq;
			int MINVq;
		}SetdqCmdLimit;

		union Data_Setting{
			struct _ByteSet {
			  byte L;
			  byte H;
			} Byte;
			int Data;
		};

		void Init();
		void doEncoder();
		void revFromMCU();
		void sendVoltCmd();
		void FbMotorData(long dT);
		void Enable();
		void Disable();

		double cmd_volt = 0.0;
		double fb_omega = 0;

		DDWheel(SetENCPin* ENC_Pin, SetMotorParam* motor_param, SetCtrlParam* Ctrl_param,SetdqCmdLimit* Limit);
	private:
		Data_Setting MotorData[3];
		byte getData[8];
		byte sendData[7] = {123, 0, 0, 0, 0, 0, 125};
		byte sendDataStop[7] = {123, 0, 0, 0, 0, 85, 125};

		SetMotorParam BLDC_DDWheel;
		SetCtrlParam DDWheel_Ctrl;

		#define left 0
		#define right 1

		//command reference
		int Vq = 0, Vd = 0;
		int Iq = 0, Id = 0;
		#define VqVdMode 0
		#define VqIdMode 1
		#define IdIqMode 2
		int InputCmd[2];
		int checksum = 0;

		// encoder
		SetENCPin Hall;
		volatile long Encoderpos = 0;
		long EncoderposPre = 0;
		volatile int lastEncoded = 0;

		int pinAState = 0, pinAStateOld = 0;
		int pinBState = 0, pinBStateOld = 0;
		int pinCState = 0, pinCStateOld = 0;
		int EncodeDiff = 0, EncodeDiffPre = 0;

		//Volt control
		const double Vq_formatRatio=1499/12; //convert voltage to Vq voltage format ( 12 (max voltage) : 1499 (Vq format max) )
		SetdqCmdLimit InputLimit;

};
