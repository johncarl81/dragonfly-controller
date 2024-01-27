#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy

class VirtualRangefinderPublisher(Node):

    MAX_ALTITUDE = 30.0

    def __init__(self, id):
        super().__init__('virtual_rangefinder')
        self.id = id
        self.create_subscription(PoseStamped, f"/{self.id}/mavros/local_position/pose", self.pose_callback, qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.publisher = self.create_publisher(Range, f"/{self.id}/mavros/rangefinder/rangefinder", 10)

    def pose_callback(self, msg):
        range_msg = Range()
        altitude = msg.pose.position.z
        range_msg.range = altitude if 0.0 < altitude <= self.MAX_ALTITUDE else 0.0
        range_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(range_msg)


def main():
    rclpy.init(args=sys.argv)

    parser = argparse.ArgumentParser(description='Starts ROS publisher for altitude-based test rangefinder.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publisher = VirtualRangefinderPublisher(args.id)

    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
