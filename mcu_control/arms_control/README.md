# andbot_arm_control
## Joint frame 
![](https://github.com/oudeis/therobot/blob/master/Andbot_pkg/andbot_arm_control/joint_frame.jpeg)
 
## status
* L0~L3 
  * position command topic : ***ready***
  * position command duration : ***ready***
  * feedback topic : not ready
* R0~R3 
  * position command topic : ***ready***
  * position command duration : ***ready***
  * feedback topic : not ready
* N0, L4, R4
  * not ready
  
## Custom mseeages
     1. JointCmd
        -float32 angle
        -float32 duration
     2. JointFb
        -float32 angle
        -uint16  current
        -bool    hall1
        -bool    hall2
     3. Calibration
        Request
        -uint8 p
        -uint8 i
        -uint8 d
        -uint8 currentlim
        -bool driverstate
        Respond
        -uint8 p
        -uint8 i
        -uint8 d
        -uint8 currentlim
        -bool driverstate

## Store config in EEPROM
     Address     Data
     0           ID_number
     1           P (Proportional)
     3           I (Intergral)
     5           D (Derivative)
     7           C_L (Current Limit, MAX = 255 * 100 = 25A)

# Calibration mode
Sending PID, current limit, and driver state as service through rosserial to driver board.

# Working mode
Sending Angle and duration through rosserial.

## programming RS485 firmware
* program position_control_master.ino into mega 2560.
* program position_control_slave.ino into 3in1 board.
  * Edit line 31 in "wiring.c": #define MICROSECONDS_PER_TIMER0_OVERFLOW (clockCyclesToMicroseconds(8 * 256))  
  * Select #define L0 ~ R3 when programming each slave devices.

# andbot_test_joint

## Step
* cd ~/catkin_ws/src/metal1
* git pull
* git checkout andbot2
* rospack profile
* roslaunch andbot andbot_arm_test.launch 
* rostopic list (following should be listed)
  * /andbot/joint/L/cmd/offset_v
  * /andbot/joint/L/cmd/position
  * /andbot/joint/R/cmd/offset_v
  * /andbot/joint/R/cmd/position
  * /andbot/left_arm/goal
  * /andbot/predefinedPoses
  * /andbot/right_arm/goal
  * /diagnostics
  * /rosout
  * /rosout_agg


* rosrun andbot_test_joint teleop_home 0 0.1
  * left arm 0 Axis & 1 Axis (speed = 0.1rad/s)
  * Press '↑' and '↓'to control the 0 Axix forward and backward, and move this Axis to 'mechanical origin point'
  * Press '←' and '→'to control the 1 Axix left and right, and move this Axis to 'mechanical origin point'
  * press 'p' to pause the mvoement
  
* rosrun andbot_test_joint teleop_home 1 0.1
  * left arm 2 Axis & 3 Axis (speed = 0.1rad/s)
  * Press '↑' and '↓'to control the 3 Axix forward and backward, and move this Axis to 'mechanical origin point'
  * Press '←' and '→'to control the 2 Axix left and right, and move this Axis to 'mechanical origin point'
  * press 'p' to pause the mvoement
  
* rosrun andbot_test_joint teleop_home 2 0.1
  * right arm 0 Axis & 1 Axis (speed = 0.1rad/s)
  * Press '↑' and '↓'to control the 0 Axix forward and backward, and move this Axis to 'mechanical origin point'
  * Press '←' and '→'to control the 1 Axix left and right, and move this Axis to 'mechanical origin point'
  * press 'p' to pause the mvoement
  
* rosrun andbot_test_joint teleop_home 3 0.1
  * right arm 2 Axis & 3 Axis (speed = 0.1rad/s)
  * Press '↑' and '↓'to control the 3 Axix forward and backward, and move this Axis to 'mechanical origin point'
  * Press '←' and '→'to control the 2 Axix left and right, and move this Axis to 'mechanical origin point'
  * press 'p' to pause the mvoement

* Finally press 'Ctrl+C' to kill this node

# andbot predefined pose (arm movement test)
* rostopic pub /andbot/predefinedPoses std_msgs/UInt8 "data: 0" (home)
* rostopic pub /andbot/predefinedPoses std_msgs/UInt8 "data: 3" (strong)
* rostopic pub /andbot/predefinedPoses std_msgs/UInt8 "data: 2" (holdtray)
