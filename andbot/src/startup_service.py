#!/usr/bin/env python

#from beginner_tutorials.srv import *
import os
from andbot.srv import *
import rospy
 
def teleop(req):
    os.system('roslaunch turtlebot_teleop keyboard_teleop.launch') 
    #return 'Killed'
   
def mapping(req):
    os.system('roslaunch andbot gmapping_ekf.launch')
    return 'Killed'

def navigation(req):
    os.system('roslaunch andbot 578.launch')
    return 'Killed'

def reboot(req):
    os.popen("sudo -S reboot", 'w').write('odroid\n')
    return 'Killed'

def shutdown(req):
    os.popen("sudo -S shutdown -h now", 'w').write('odroid\n')
    return 'Killed'
 
def startup_server():
    rospy.init_node('startup_service')
    s1 = rospy.Service('teleop_service', Startup, teleop)
    s2 = rospy.Service('mapping_service', Startup, mapping)
    s3 = rospy.Service('navigation_service', Startup, navigation)
    s4 = rospy.Service('reboot_service', Startup, reboot)
    s5 = rospy.Service('shutdown_service', Startup, shutdown)
    print "Waiting for services to be called."
    rospy.spin()
   
if __name__ == "__main__":
    startup_server()
