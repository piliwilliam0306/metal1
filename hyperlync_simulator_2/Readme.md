#Hyperlynch Mobile Robot Simulator 2.0

Hyperlync Robot Simulator implementation under GAZEBO

#How to install

* Be sure to have gazebo and all its indigo packages already installed
* Copy this package under your **catkin_ws/src/** workspace
* Edit **my_map2.yaml** and fix the path of **my_map2.pgm** relative to your laptop
* Edit **move_base.launch** and fix the path of **my_map2.yaml** relative to your laptop
* In the **catkin_ws** folder run **catkin_make clean**
* In the **catkin_ws** folder run **catkin_make --pkg hyperlync_robot**
* In the **catkin_ws** folder run **catkin_make --pkg hyperlync_teleop**
* You have to install the following packages
* *sudo apt-get install aptitude*
* *sudo apt-get install ros-indigo-joy*
* *sudo aptitude install ros-indigo-gazebo-ros-control*
* *sudo aptitude install ros-indigo-joint-state-controller*
* *sudo aptitude install ros-indigo-effort-controllers*
* *sudo aptitude install ros-indigo-laser-proc*
* *sudo aptitude install ros-indigo-gmapping*
* *sudo aptitude install ros-indigo-openslam-gmapping*

#How to build and save a map
* [IMPORTANT] Before following roslaunch commands, plz make sure the **ROS_IP** environment variable are set in all terminals
 * **export ROS_IP=x.x.x.x**   // x.x.x.x is the IP address of your ROS machine 
* Run **roslaunch hyperlync_robot hyperlync_simulator.launch** for the maze world and wait a bit, or
* Run **roslaunch hyperlync_robot hyperlync_simulator_apartment.launch** for the apartment world and wait a bit
* If you get errors like *Unable to set value [0,100000001] ...* stop the app, write *export LC_NUMERIC=C* and restart the launch
* Connect your joypad and run **roslaunch hyperlync_teleop xbox360_teleop_andbot.launch**
 * Push the right frontal button labeled as "RB" to activate cmd_vel publishing. Move the left stick around to control the velocity and the "A" button to rotate the laser scanner 
* If you dont have xbox360 joypad, you can use following node to teleop the robot by PC keyboard.

          $ rosrun hyperlync_teleop hyperlync_teleop_key1
          Arrow UP:    go forward
          Arrow DOWN:  go backward
          Arrow LEFT:  turn left
          Arrow RIGHT: turn right
          P key:       stop moving
          A:           start laser scan
          S:           stop laser scan

* Look at the **robot_mapping.launch** file, if you wanna use the single beam you have to set *arg name="scan_topic" default="scan"* , otherwise if you want the wide scan *arg name="scan_topic" default="laser"*
* Run **roslaunch hyperlync_robot robot_mapping.launch** for the laser scan mapping
* Run **roslaunch hyperlync_robot robot_mapping_sonar.launch** for the sonar scan mapping
* Run **rosrun hyperlync_robot rotating_laser** for the laser scan mapping
* Run **rosrun hyperlync_robot scan_from_sonar** for the sonar scan mapping
* If you are using the wide laser scans you can navigate within the room using the left stick of the joypad (remember to push the safety button on the top right and the A or X button of the XBOX joypad to rotate the laser scanner base)
* If you are using the single beam laser scans you have to stay motionless, get the 180° scan pushing the rotating base button, release the rotating base button of the joypad and only after that move (not so much) the robot to another point of the room to complete the other 180° scan of what is around it.
* After the full navigation you can save the map running **rosrun map_server map_saver -f /YOURCATKINWS/src/hyperlync_simulator_2/hyperlync_robot/NAMEOFYOURMAP**

#How to run the autonomous navigation

* [IMPORTANT] Before following roslaunch commands, plz make sure the **ROS_IP** environment variable are set in all terminals
 * **export ROS_IP=x.x.x.x**                  // x.x.x.x is the IP address of your ROS machine 
* Run **roslaunch hyperlync_robot hyperlync_simulator.launch**
* Run **roslaunch hyperlync_robot move_base.launch**
* Go on Rviz, click on **2D Nav Goal** and point and click on the Rviz map as goal for the robot

<br><br><br>

All the files are edited or created by: <br>
Dott. Antonio Mauro Galiano<br>
**antoniomauro.galiano@gmail.com**<br>
Zach<br>
**qoogood1234@gmail.com**<br>
Kevin Kuei<br>
**kevin.kuei.0321@gmail.com**<br>
Only for simulation purpose<br>
