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
