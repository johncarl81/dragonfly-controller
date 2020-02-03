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
from mavros_msgs.msg import WaypointReached
from mavros_msgs.msg import CommandCode
from sensor_msgs.msg import NavSatFix

def calculateLatitude(latitude, offset):
    return latitude + (offset * 0.00000898)

def calculateLongitude(latitude, longitude, offset):
    return longitude + (offset * 0.00000898) / math.cos(latitude * 0.01745)

def createWaypoint(lat, lon, altitude, type):
    waypoint = Waypoint()
    waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    waypoint.command = type
    waypoint.is_current = 0
    waypoint.autocontinue = 0
    waypoint.x_lat = lat
    waypoint.y_long = lon
    waypoint.z_alt = altitude
    waypoint.param1 = 1

    return waypoint

def buildDDSAWaypoints(centerx, centery, altitude, size, index, loops, radius):

  waypoints = []
  start = createWaypoint(centerx, centery, altitude, CommandCode.NAV_WAYPOINT)
  start.is_current = 1
  waypoints.append(start)
  for loop in range(0, loops):
    for corner in range(0, 4):
      xoffset = 0
      yoffset = 0

      if (loop == 0 and corner == 0):
        xoffset = 0
        yoffset = index + 1
      else:
        xoffset = 1 + index + (loop * size)
        yoffset = xoffset
        if (corner == 0):
          xoffset = -(1 + index + ((loop - 1) * size))
        elif (corner == 3):
          xoffset = -xoffset
        if (corner == 2 or corner == 3):
          yoffset = -yoffset
 
      latitude = calculateLatitude(centerx, xoffset * radius)
      longitude = calculateLongitude(centerx, centery, yoffset * radius)
      waypoint = createWaypoint(latitude, longitude, altitude, CommandCode.NAV_WAYPOINT)
      waypoints.append(waypoint)
  waypoints.append(start)
  return waypoints



rospy.init_node('ddsa_service')
rospy.wait_for_service('JUAV1/mavros/set_mode')
rospy.wait_for_service('JUAV1/mavros/mission/clear')
rospy.wait_for_service('JUAV1/mavros/mission/push')

setmode_service = rospy.ServiceProxy('JUAV1/mavros/set_mode', SetMode)
wp_clear_srv = rospy.ServiceProxy('JUAV1/mavros/mission/clear', WaypointClear)
wp_push_srv = rospy.ServiceProxy('JUAV1/mavros/mission/push', WaypointPush)

print "Waypoint clear"
print wp_clear_srv()

time.sleep(5)

def logWaypoint(waypoint):
    print "Waypoint: ", waypoint.wp_seq

rospy.Subscriber('JUAV1/mavros/mission/reached', WaypointReached, logWaypoint)

def updatePosition(position):
    position_update.unregister()

    print "Position: ", position.latitude, " ", position.longitude

    altitude = 3

    waypoints = buildDDSAWaypoints(position.latitude, position.longitude, altitude, 1, 0, 1, 1)

    print "Push waypoints"
    print wp_push_srv(start_index=0, waypoints=waypoints)

    time.sleep(10)

    print "Set Mode"
    print setmode_service(custom_mode = "AUTO")	

    print "Commanded"

position_update = rospy.Subscriber('JUAV1/mavros/global_position/global', NavSatFix, updatePosition)

rospy.spin()




