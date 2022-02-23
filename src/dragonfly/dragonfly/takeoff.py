#!/usr/bin/env python
import argparse
import time

import rclpy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode


def takeoff(id):
    rclpy.init(args=id)
    node = rclpy.create_node('takeoff_service')

    setmode_service = node.create_client(SetMode, "{}/mavros/set_mode".format(id))
    while not setmode_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('setmode_service service not available, waiting again...')

    arm_service = node.create_client(CommandBool, "{}/mavros/cmd/arming".format(id))
    while not arm_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('arm_service service not available, waiting again...')

    takeoff_service = node.create_client(CommandTOL, "{}/mavros/cmd/takeoff".format(id))
    while not takeoff_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('takeoff_service service not available, waiting again...')

    print("Setup complete")

    print("Set Mode")
    print(setmode_service.call(SetMode.Request(custom_mode="STABILIZE")))

    time.sleep(1)

    print("Arming")
    print(arm_service.call(True))

    time.sleep(5)

    print("Change to Guided")

    print(setmode_service.call(SetMode.Request(custom_mode="GUIDED")))

    print("Take off")
    print(takeoff_service.call(CommandTOL.Request(altitude=3)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Command a drone to takeoff.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    takeoff(args.id)
