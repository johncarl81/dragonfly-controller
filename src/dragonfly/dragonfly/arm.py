#!/usr/bin/env python3
import argparse
import time

import rclpy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode


def arm(id):
    rclpy.init(args=id)
    node = rclpy.create_node('arm_test_service')
    logger = node.get_logger()

    setmode_service = node.create_client(SetMode, f"{id}/mavros/set_mode")
    while not setmode_service.wait_for_service(timeout_sec=1.0):
        logger.info('service not available, waiting again...')

    arm_service = node.create_client(CommandBool, f"{id}/mavros/cmd/arming")
    while not arm_service.wait_for_service(timeout_sec=1.0):
        logger.info('service not available, waiting again...')

    logger.info("Setup complete")

    logger.info("Set Mode")
    logger.info(setmode_service.call(SetMode.Request(custom_mode="STABILIZE")))

    time.sleep(1)

    logger.info("Arming")
    logger.info(arm_service.call(True))

    time.sleep(5)

    logger.info("Disarming")
    logger.info(arm_service.call(False))

    logger.info("Commanded")


if __name__ == '__main__':
    # Get RGB colors from command line arguments.
    parser = argparse.ArgumentParser(description='Arm a drone.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    arm(args.id)
