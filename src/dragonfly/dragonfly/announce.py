#!/usr/bin/env python3

import argparse
import sys

import rclpy
from std_msgs.msg import String

class Announcer:

    def __init__(self, id, node):
        self.id = id
        self.node = node

        self.node.get_logger().info("publishing name {} on /dragonfly/announce".format(self.id))
        self.pub = self.node.create_publisher(String, "/dragonfly/announce", 10)

    def announce(self):
        msg = String()
        msg.data = self.id
        self.pub.publish(msg)


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('name_announcement')

    parser = argparse.ArgumentParser(description='Starts ROS publisher for name announcement.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    announcer = Announcer(args.id, node)

    node.create_timer(1, announcer.announce)

    rclpy.spin(node)

if __name__ == '__main__':
    main()
