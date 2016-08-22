# Rugby_head_control

## Programming firmware
* program rugby_head_master.ino into mega 2560.
* program rugby_head_slave.ino into 3in1 board.
  * Edit line 31 in "wiring.c": #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256))  
  * Select #define Pan or Tilt when programming each slave devices.

## Running Serial Node and check if all topics appears
* rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=1000000
* rostopic list (following should be listed)
  * /rugby/Head/cmd/offset_v
  * /rugby/Head/cmd/position
  
## Joint Calibration
* cd ~/catkin_ws/src/metal1
* git pull
* git checkout andbot2
* cd ~/catkin_ws/
* catkin_make
* rospack profile
* rosrun andbot teleop_rugby 0.1
  * Rugby Head 0 Axis & 1 Axis (speed = 0.1rad/s)
  * Press '↑' and '↓'to rotate up and down.
  * Press '←' and '→'to rotate left and right.
  * press 'p' to pause the mvoement
  * press 'Ctrl+C' to kill this node
  
## Joint Angle test
* rostopic pub /rugby/Head/cmd/position andbot/HeadCmd "angle1: 0.0 angle2: 0.0" (pan, tilt angle in radian)
