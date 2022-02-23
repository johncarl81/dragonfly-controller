#!/usr/bin/env python

import argparse
import sys
import threading

import rclpy
from std_msgs.msg import String


def publishco2(id, node):
    node.get_logger().info("publishing name {} on /dragonfly/announce".format(id))
    pub = node.create_publisher(String, "/dragonfly/announce", 10)

    rate = node.create_rate(1)
    while rclpy.ok():
        msg = String()
        msg.data = id
        pub.publish(msg)
        rate.sleep()


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('name_announcement')

    parser = argparse.ArgumentParser(description='Starts ROS publisher for name announcement.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    try:
        publishco2(args.id, node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
