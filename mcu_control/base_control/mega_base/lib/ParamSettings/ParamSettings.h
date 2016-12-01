/*
 * =====================================================================================
 *
 *       Filename:  ParamsSettings.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2016年12月01日 21時28分36秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Weber-Hsu
 *        Company:  Advanced Robotics Corporation
 *
 * =====================================================================================
 */

/* *****************************************************/

//#define ANDBOT 1
//#define RUGBY 2
#define ANGELBOT 3

//Sensor Pin
const int TrigPin1 = 30;  //PC7
const int TrigPin2 = 31;  //PC6
const int TrigPin3 = 32;  //PC5
const int TrigPin4 = 33;  //PC4
const int TrigPin5 = 34;  //PC3
const int TrigPin6 = 35;  //PC2
const int TrigPin7 = 36;  //PC1
const int TrigPin8 = 37;  //PC0

const int EchoPin1 = 42;  //PL7
const int EchoPin2 = 43;  //PL6
const int EchoPin3 = 44;  //PL5
const int EchoPin4 = 45;  //PL4
const int EchoPin5 = 46;  //PL3
const int EchoPin6 = 47;  //PL2
const int EchoPin7 = 48;  //PL1
const int EchoPin8 = 49;  //PL0

bool bump1_reading;
bool bump2_reading;
bool bump3_reading;
bool bump4_reading;
bool cliff1_reading;
bool cliff2_reading;
bool cliff3_reading;
bool cliff4_reading;

// robot param
double WheelRadius = 0.0375, WheelSeparation = 0.247;

// motor parameters
#if defined (ANDBOT)
        #define MaxSpeed 10.96
#elif (ANGELBOT)
        #define MaxSpeed 31
#else
        #define MaxSpeed 31
#endif
/* ************  End of declarations for ROS usages ****************/

