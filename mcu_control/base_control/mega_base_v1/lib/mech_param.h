if defined (ANDBOT)
	double MaxSpeed = 10.96;
#else
	double MaxSpeed = 31;
#endif

bool driverEn;
#define Volt_MAX 12
#define Volt_MIN -12
#define Vq_MAX 1000 //  ~8V
#define Vq_MIN -1000 // ~-8V

//Pin assignments
const int TrigPin1 = 30;  //PC7
const int TrigPin2 = 31;  //PC6
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

// motor parameters
// int CPR = 90;                                                                     //encoder count per revolution
// int gear_ratio = 1;
// const double MAXAngularSpeed = 47.1238898 ;//  450 / 60 * 2 * PI => DD motor nominal rotation speed: 450 rpm
//
//
// #define Axis_left 0
// #define Axis_right 1
//
// #define ENCDiffMode 0
// #define ENCMovFilterMode 1
// #define ENCOutputMode ENCMovFilterMode
//
// //cmd ref
// #define VqVdMode 0
// #define VqIdMode 1
// #define IdIqMode 2
// #define dqCmdMode VqVdMode
//
// // robot param
// double wheelRadius = 0.085, wheelSeparation = 0.432;
// /*  ************  End of declarations for ROS usages ****************/ */
