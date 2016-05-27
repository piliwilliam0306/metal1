# Base Test Proceduce

## Modify Arduino PWM Frequency 
  * Overwrite wiring.c in: 
     ~/arduino-1.6.5/hardware/arduino/avr/cores/arduino
  * Edit line 31 in "wiring.c"     
  * Mega Board:
     #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(64 * 256))
  * Vnh5019 Board:
     #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256))

## Uploading mega code
  * Copy "ros_lib" folder into "Arduino/libraries" folder.
  * "mega_base_ultrasonic_v1" is for base mega board.
  * Select Mega2560 when uploading.

## Uploading vnh5019 code
  * "vnh5019_andbot_test.ino" is to test if motor can achieve desire speed. (Speed control test)
  * "#define WHEEL_TYPE RIGHT_WHEEL" when uploading for right wheel. 
  * "#define WHEEL_TYPE LEFT_WHEEL" when uploading for left wheel.
  * Enter 4 charactors target speed in Arduino Serial Window. (Ex: 2.00, -2.0, -4.0)
  * If "actual speed" is close to "target speed," wiring and control board are working; if not, check wiring or swap encoder pins.
  * "vnh5019.ino" is for the motor controller board for base. (Upload this if vnh5019 control board pass speed test.)
  * Select Arduino Pro Mini when uploading.

## Check Odroid IP adress
     $ ifconfig

## Compiling packages (in Terminal)
     $ cd ~/catkin_ws/src
     $ git clone https://github.com/piliwilliam0306/metal1.git
     $ cd ~/catkin_ws
     $ catkin_make
     $ rospack profile
     
## Test Rosserial
     $ roscore
     $ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600
     $ rostopic pub /cmd_wheel_angularVel andbot/WheelCmd 
     "speed1: 2.0
     mode1: true
     speed2: 2.0
     mode2: true" 
     $ rostopic echo feedback_wheel_angularVel
     * if "feedback_wheel_angularVel" topic output is close to "cmd_wheel_angularVel," rosserial and motor control board is working.
     $ Ctrl + C

## Make robot move
     $ roslaunch andbot andbot_v1.launch
  * Type Odroid IP address into Android App to Teleop. 
  * You should see map updating after moving the robot for a while.
     
