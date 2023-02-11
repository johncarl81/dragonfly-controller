#!/usr/bin/env python3
import argparse

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode
from rclpy.qos import QoSProfile

node = None
position_update = None


def land(id):
    global node, position_update
    rclpy.init(args=id)
    node = rclpy.create_node('land_service')

    land_service = node.create_client(CommandTOL, f"{id}/mavros/cmd/land")
    while not land_service.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')
    print("Landing")
    resp = land_service.call_async(CommandTOL.Request(altitude=0))
    rclpy.spin_until_future_complete(node, resp)
    print(resp)
    position_update = node.create_subscription(State, f"{id}/mavros/state", updateState,
                                               qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))


def updateState(state):
    global node, position_update
    node = rclpy.create_node('land_service')
    if not state.armed:
        print("Landed.")
        print(f"State: {state}")

        setmode_service = node.create_client(SetMode, f"{id}/mavros/set_mode")
        print("Set Mode")
        print(setmode_service.call(SetMode.Request(custom_mode="STABILIZE")))

        position_update.destroy()

    print("Waiting for landing...")
    position_update = node.create_subscription(State, f"{id}/mavros/state", updateState,
                                               qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

    rclpy.spin(node)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Command a drone to land.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()
    land(args.id)
