#!/usr/bin/env python
import argparse
import math
import time

import rclpy
from mavros_msgs.msg import CommandCode
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointReached
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import WaypointClear
from mavros_msgs.srv import WaypointPush
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix

setmode_service = None
wp_push_srv = None
wp_clear_srv = None


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
    global setmode_service, wp_clear_srv, wp_push_srv
    rclpy.init(args=id)
    node = rclpy.create_node('ddsa_service')

    setmode_service = node.create_client(SetMode, "{}/mavros/set_mode".format(id))
    while not setmode_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('set_mode service not available, waiting again...')

    wp_clear_srv = node.create_client(WaypointClear, "{}/mavros/mission/clear".format(id))
    while not wp_clear_srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('clear service not available, waiting again...')

    wp_push_srv = node.create_client(WaypointPush, "{}/mavros/mission/push".format(id))
    while not wp_push_srv.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('push service not available, waiting again...')

    print("Waypoint clear")
    wp_clear = wp_clear_srv.call_async(WaypointClear())  # @TODO try None if this does not work
    print(wp_clear)
    rclpy.spin_until_future_complete(node, wp_clear)

    time.sleep(5)

    def logWaypoint(waypoint):
        print("Waypoint: {}".format(waypoint.wp_seq))
        node.create_subscription(WaypointReached, "{}/mavros/mission/reached".format(id), logWaypoint,
                                 qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

    def updatePosition(position):
        global setmode_service, wp_clear_srv, wp_push_srv
        position_update.destroy()

        print("Position: {} {}".format(position.latitude, position.longitude))

        altitude = 3

        waypoints = buildDDSAWaypoints(position.latitude, position.longitude, altitude, 1, 0, 5, 1)

        wp_push = wp_push_srv.call_async(WaypointPush.Request(start_index=0, waypoints=waypoints))
        print("Push waypoints")
        print(wp_push)
        rclpy.spin_until_future_complete(node, wp_push)

        time.sleep(10)

        print("Set Mode")
        print(setmode_service())

        setmode_service = setmode_service.call_async(custom_mode="AUTO")
        rclpy.spin_until_future_complete(node, setmode_service)

        print("Commanded")

    position_update = node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(id), updatePosition,
                                               qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))
    rclpy.spin(node)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Command a drone to follow the DDSA.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    ddsa(args.id)
