#!/usr/bin/env python
import argparse

import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix

position = None
node = None


def callback(data):
    global position, node
    node.get_logger().info("I heard %s", data)


def listener(id):
    global node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rclpy.init(args=id)
    node = rclpy.create_node('gpslistener')
    node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(id), callback,
                             qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

    # spin() simply keeps python from exiting until this node is stopped
    rclpy.spin(node)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Listen to and print the given drone\'s GPS.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    listener(args.id)
