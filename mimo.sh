#!/bin/bash

* roscore
* ssh odroid@metal1
* rosrun rosserial_python serial_node.py _port:=/dev/mega_base _baud:=115200
* rosrun andbot mimo_base_controller



# host
* expor ROS_MASTER_URI='http://metal1:11311'
* listen speed
  * rostopic echo /feedback_wheel_angularVel
* teleop control
  * rosrun andbot_teleop andbot_teleop_key
* burn mega code 
  * platformio run -t upload --upload-port=/dev/mega_base

