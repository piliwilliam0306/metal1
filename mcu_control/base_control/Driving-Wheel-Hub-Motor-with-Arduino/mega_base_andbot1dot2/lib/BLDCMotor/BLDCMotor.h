/*
 * =====================================================================================
 *
 *       Filename:  BLDCMotor.h
 *
 *    Description:  This is a header file for BLDC control library.
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
#ifndef BLDCMotor_h
#define BLDCMotor_h

#if ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#endif

class BLDCMotor {
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
			int pinC;
		};

		struct SetCtrlParam{
			int axis;
			int mode;
		};

		struct dqCmd{
			int Vq;
			int Vd;
			int Iq;
			int Id;
			int InputCmd[1];
		};

		struct CmdRef{
			double VoltCmd = 0.0;
			double CurrentCmd = 0.0;
		}CmdRef;

		struct FbMotorInfo{
			double AngularVel = 0.0;
		}FbMotorInfo;

		typedef struct{
			int MAXVoltCmd;
			int MINVoltCmd;
			int MAXVq;
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
		void SendCmd();
		void GetMotorData(long dT,int mode);
		void Enable();
		void Disable();
		void dqInputSelect(int dCmd, int qCmd);
		void SerialSend2Driver(byte* sendData, int axis);
		int SerialGetDriverData(int axis);

		volatile int Encoderpos = 0;
		int curEncoderpos = 0;
		int EncoderposPre = 0;

		BLDCMotor(SetENCPin* ENC_Pin, SetMotorParam* motor_param, SetCtrlParam* Ctrl_param,SetdqCmdLimit* Limit);
	private:
		Data_Setting MotorData[3];
		byte getData[8];
		byte sendData[7] = {123, 0, 0, 0, 0, 0, 125};
		byte sendDataStop[7] = {123, 0, 0, 0, 0, 85, 125};

		SetMotorParam BLDCParam;
		SetCtrlParam BLDCCtrl;
		SetdqCmdLimit InputLimit;

		#define left 0
		#define right 1

		// encoder
		#define ENCDiffMode 0
		#define ENCMovFilterMode 1
		struct ENCMovFilter{
			int count = 0;
			int Samples[500] = {0};
			int sum;
			double Output;
		}ENCMovFilter;
		SetENCPin Hall;

		volatile int lastEncoded = 0;

		int pinAState = 0, pinAStateOld = 0;
		int pinBState = 0, pinBStateOld = 0;
		int pinCState = 0, pinCStateOld = 0;
		int EncodeDiff = 0, EncodeDiffPre = 0;

		int SetENCMovFilter(int Total, long dT);

		//Volt control
		dqCmd dqCmd;
		const double Vq_formatRatio=1499/12; //convert voltage to Vq voltage format ( 12 (max voltage) : 1499 (Vq format max) )
		//command reference
		#define VqVdMode 0
		#define VqIdMode 1
		#define IqIdMode 2
		int InputCmd[2];
		int checksum = 0;
};
