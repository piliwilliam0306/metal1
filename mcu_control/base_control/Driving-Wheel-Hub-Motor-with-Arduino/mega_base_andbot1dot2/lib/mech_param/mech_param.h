/*
 * =====================================================================================
 *
 *       Filename:  mech_param.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2016年11月17日 09時34分56秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  Weber-Hsu
 *   Organization:  Advanced Robotics Corporation
 *
 * =====================================================================================
 */

/* *****************************************************/

/*  define  ROS node and topics */
bool driverEn;
#define Volt_MAX 12
#define Volt_MIN -12
#define Vq_MAX 1000 //  ~8V
#define Vq_MIN -1000 // ~-8V

// motor parameters
int CPR = 90;                                                                     //encoder count per revolution
int gear_ratio = 1;
const double MAXAngularSpeed = 47.1238898 ;//  450 / 60 * 2 * PI => DD motor nominal rotation speed: 450 rpm


#define Axis_left 0
#define Axis_right 1

//encoder pin assignment
//left wheel
#define enc_pinA_left 2
#define enc_pinB_left 3
#define enc_pinC_left 21
//right wheel
#define enc_pinA_right 20
#define enc_pinB_right 19
#define enc_pinC_right 18

#define ENCDiffMode 0
#define ENCMovFilterMode 1
#define ENCOutputMode ENCMovFilterMode

//cmd ref
#define VqVdMode 0
#define VqIdMode 1
#define IdIqMode 2
#define dqCmdMode VqVdMode

// robot param
double wheelRadius = 0.085, wheelSeparation = 0.432;
/* ************  End of declarations for ROS usages ****************/
