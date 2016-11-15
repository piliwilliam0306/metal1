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
        DDWheel(int pinA, int pinB, int pinC, int ind);

        void Init();
        void doEncoder();
        void revFromMCU();
        void sendVoltCmd();
        void FbMotorData();
        void Enable();
        void Disable();
private:
//		const int CALIBRATION_SAMPLES = 1000;
//		const int READ_SAMPLES = 30;
//
//		/* sampling timing of output pulse in Dust sensor (from Datasheet)*/
//		const int samplingTime = 280; // LED Pulse Width = samplingTime + deltaTime = 320us
//		const int deltaTime = 40;
//		const int sleepTime = 9680; // period (per pulse) = 10ms, i.e, sleepingTime = 10ms - 320us = 9680 us

        union Data_Setting{
        struct _ByteSet {
              byte L;
              byte H;
            } Byte;
            int Data;
        };
        byte getData;
        byte sendData[7] = {123, 0, 0, 0, 0, 0, 125};
        byte sendDataStop[7] = {123, 0, 0, 0, 0, 85, 125};

        int Wheel_select = 0;
        #define left 0
        #define right 1

        //command
        int Vq = 0, Vd = 0, CheckSum = 0;

        // encoder
        const int encoderPinA;
        const int encoderPinB;
        const int encoderPinC;

        volatile long Encoderpos = 0;
        long EncoderposPre = 0;
        volatile int lastEncoded = 0;

        int pinAState = 0, pinAStateOld = 0;
        int pinBState = 0, pinBStateOld = 0;
        int pinCState = 0, pinCStateOld = 0;
        int EncodeDiff = 0, EncodeDiffPre = 0;

        //Volt control
        double volt_target = 0.0;
        double fb_omega = 0;
};
