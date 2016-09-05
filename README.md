# Instructions for fundamental testing on angel 
## Table of contents
* System overview
  * Diagrams
  * Overall abstract diagram
  * Wiring diagram of modules
* Component test (mech-electro) (non ROS-based)
  * Motor driving test
    * Setup requirements (specifications and schemetics for each item included)
      * motor 
      * driver board 
      * communication board
    * Setup wiring diagram
    * Firmware upload instruction
      * Firmware version check
      * Serial port & baud rate configuration
    * Begin Testing 
* Module test (either or not ROS-based)
  * Base driving test
    * Setup requirements 
      * motor-control module
      * mega2560
      * Odroid XU4 or PC
        * roscore & rosserial 
    * Setup wiring diagram
    * Firmware upload instruction
      * Firmware version check
      * rosserial port & baud rate configuration 
    * Begin Testing
      * Left/right Wheel
  * IMU sensing test
    * Setup requirements 
      *  mpu6050
      *  promini
    * Setup wiring diagram
    * Firmware upload instruction
      * Firmware version check
      * Serial port & baud rate configuration 
    * Begin Testing
* Function test (ROS-based)
  * Teleoperation
    * roslaunch scripts
* Appendix
  * ROS environment setup
    * ROS MASTER
    * ROS IP 

##NOTE
  * use "andbot.launch" when working with plastic model.
  * use "metal.launch" when working with metal model.


## TF_TREE for wheel odom only
![](https://github.com/piliwilliam0306/metal1/blob/master/andbot_launch.png)

## TF_TREE for odom combined
![](https://github.com/piliwilliam0306/metal1/blob/master/andbot_ekf_launch.png)
