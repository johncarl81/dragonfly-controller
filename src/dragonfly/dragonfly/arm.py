#!/usr/bin/env python
import argparse
import time

import rclpy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode


def arm(id):
    rclpy.init(args=id)
    node = rclpy.create_node('arm_test_service')

    setmode_service = node.create_client(SetMode, "{}/mavros/set_mode".format(id))
    while not setmode_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    arm_service = node.create_client(CommandBool, "{}/mavros/cmd/arming".format(id))
    while not arm_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    print("Setup complete")

    print("Set Mode")
    print(setmode_service.call(SetMode.Request(custom_mode="STABILIZE")))

    time.sleep(1)

    print("Arming")
    print(arm_service.call(True))

    time.sleep(5)

    print("Disarming")
    print(arm_service.call(False))

    print("Commanded")


if __name__ == '__main__':
    # Get RGB colors from command line arguments.
    parser = argparse.ArgumentParser(description='Arm a drone.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    arm(args.id)
