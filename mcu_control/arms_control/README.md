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
  
## Store config in EEPROM
     Address     Data
     0           ID_number
     1           P_H (Proportional High Byte)
     2           P_L (Proportional Low Byte)
     3           I_H (Intergral High Byte)
     4           I_L (Intergral Low Byte)
     5           D_H (Derivative High Byte)
     6           D_L (Derivative low Byte)
     7           C_L (Current Limit, MAX = 255 * 100 = 25A)
