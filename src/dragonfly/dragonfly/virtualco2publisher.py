#!/usr/bin/env python

import argparse
import math
import sys

import rclpy
import std_msgs
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class VirtualCO2Publisher:
    VIRTUAL_SOURCE = dotdict({
        "latitude": 35.19465,
        "longitude": -106.59625
    })

    def __init__(self, id, node):
        self.id = id
        self.pub = node.create_publisher(String, "{}/co2".format(id), 10)
        self.node = node

    def differenceInMeters(self, one, two):
        earthCircumference = 40008000
        return [
            ((one.longitude - two.longitude) * (earthCircumference / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (earthCircumference / 360))
        ]

    def calculateCO2(self, position):

        [y, x] = self.differenceInMeters(position, self.VIRTUAL_SOURCE)

        if x >= 0:
            return 420

        Q = 5000
        K = 2
        H = 2
        u = 1

        value = (Q / (2 * math.pi * K * -x)) * math.exp(- (u * ((pow(y, 2) + pow(H, 2))) / (4 * K * -x)))

        if value < 0:
            return 420
        else:
            return 420 + value

    def position_callback(self, data):

        co2 = self.calculateCO2(data)
        # @TODO add this back was getting TypeError: __init__() takes 1 positional argument but 2 were given
        # [ros2run]: Process exited with failure 1
        #self.pub.publish(std_msgs.msg._string.String("M 55146 52516 10 55.0 0.0 0.0 800 55.0 55.0 00"))  # String("M 55146 52516 10 55.0 0.0 0.0 800 55.0 55.0 00")
        # self.pub.publish(String("M 55146 52516 {} 55.0 0.0 0.0 800 55.0 55.0 00".format(co2)))

    def publish(self):
        self.node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(self.id),
                                      self.position_callback,
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))


def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('virtual_co2')

    parser = argparse.ArgumentParser(description='Starts ROS publisher for CO2 sensor.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publisher = VirtualCO2Publisher(args.id, node)

    publisher.publish()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
