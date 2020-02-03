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

rospy.init_node('land_service')
rospy.wait_for_service('JUAV1/mavros/cmd/land')

land_service = rospy.ServiceProxy('JUAV1/mavros/cmd/land', CommandTOL)

print "Landing"
print land_service(altitude = 0)
time.sleep(30)

