#!/usr/bin/env python
import rospy
import serial
import time
import math
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import WaypointClear
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import CommandCode
from sensor_msgs.msg import NavSatFix

rospy.init_node('takeoff_service')
rospy.wait_for_service('JUAV1/mavros/set_mode')
rospy.wait_for_service('JUAV1/mavros/cmd/arming')
rospy.wait_for_service('JUAV1/mavros/cmd/takeoff')

setmode_service = rospy.ServiceProxy('JUAV1/mavros/set_mode', SetMode)
arm_service = rospy.ServiceProxy('JUAV1/mavros/cmd/arming', CommandBool)
takeoff_service = rospy.ServiceProxy('JUAV1/mavros/cmd/takeoff', CommandTOL)
print "Setup complete"

print "Set Mode"
print setmode_service(custom_mode = "GUIDED")	

time.sleep(1) 

print "Arming"
print arm_service(True)

time.sleep(1)
 
print "Take off"
print takeoff_service(altitude = 3)

time.sleep(60)




