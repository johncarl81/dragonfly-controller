#!/usr/bin/env python
import argparse
import math
import rospy
import time
from mavros_msgs.msg import CommandCode
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointReached
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointClear
from mavros_msgs.srv import WaypointPush
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


def ddsa(id):
    rospy.init_node('ddsa_service')
    rospy.wait_for_service("{}/mavros/set_mode".format(id))
    rospy.wait_for_service("{}/mavros/mission/clear".format(id))
    rospy.wait_for_service("{}/mavros/mission/push".format(id))

    setmode_service = rospy.ServiceProxy("{}/mavros/set_mode".format(id), SetMode)
    wp_clear_srv = rospy.ServiceProxy("{}/mavros/mission/clear".format(id), WaypointClear)
    wp_push_srv = rospy.ServiceProxy("{}/mavros/mission/push".format(id), WaypointPush)

    print("Waypoint clear")
    print(wp_clear_srv())

    time.sleep(5)

    def logWaypoint(waypoint):
        print("Waypoint: {}".format(waypoint.wp_seq))

    rospy.Subscriber("{}/mavros/mission/reached".format(id), WaypointReached, logWaypoint)

    def updatePosition(position):
        position_update.destroy()

        print("Position: {} {}".format(position.latitude, position.longitude))

        altitude = 3

        waypoints = buildDDSAWaypoints(position.latitude, position.longitude, altitude, 1, 0, 5, 1)

        print("Push waypoints")
        print(wp_push_srv(start_index=0, waypoints=waypoints))

        time.sleep(10)

        print("Set Mode")
        print(setmode_service(custom_mode="AUTO"))

        print("Commanded")

    position_update = rospy.Subscriber("{}/mavros/global_position/global".format(id), NavSatFix, updatePosition)

    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Command a drone to follow the DDSA.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    ddsa(args.id)
