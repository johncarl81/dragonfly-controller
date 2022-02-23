#!/usr/bin/env python
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CommandCode
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointReached
from mavros_msgs.srv import SetMode
from rclpy.qos import QoSProfile
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

            waypoint = createWaypoint(xoffset * radius, yoffset * radius, altitude, CommandCode.NAV_WAYPOINT)
            waypoints.append(waypoint)
    waypoints.append(start)
    return waypoints


def setpoint(id):
    rclpy.init(args=id)
    node = rclpy.create_node('guide_service')

    def logWaypoint(waypoint):
        print("Waypoint: {}".format(waypoint.wp_seq))

    node.create_subscription(WaypointReached, "{}/mavros/mission/reached".format(id), logWaypoint,
                             qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

    print("Change to Guided")
    setmode_service = node.create_client(SetMode, "{}/mavros/set_mode".format(id))
    print(setmode_service.call(SetMode.Request(custom_mode="GUIDED")))

    def updatePosition(position):
        position_update.destroy()
        print("Position: {} {}".format(position.latitude, position.longitude))

        setposition_publisher = node.create_publisher(PoseStamped, "{}/mavros/setpoint_position/local".format(id),
                                                      qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))
        time.sleep(.5)  # @TODO fix rostime.wallsleep(0.5)
        waypoints = buildDDSAWaypoints(0, 0, 10, 1, 0, 5, 1)

        for waypoint in waypoints:
            print("Hit enter to proceed")
            input("Enter:")
            goalPos = PoseStamped()
            goalPos.pose.position.x = waypoint.x_lat
            goalPos.pose.position.y = waypoint.y_long
            goalPos.pose.position.z = 5

            print("Going to: {}".format(goalPos))

            print(setposition_publisher.publish(goalPos))

            print("Commanded")

    position_update = node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(id), updatePosition,
                                               qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

    rclpy.spin(node)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Command a drone to point.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    setpoint(args.id)
