#!/usr/bin/env python3

import argparse
import math
import sys

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from dragonfly_messages.msg import CO2


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
        self.pub = node.create_publisher(CO2, f"{id}/co2", 10)
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
            return 420.0

        # Simple gaussian plume model adapted from: https://epubs.siam.org/doi/pdf/10.1137/10080991X
        # See equation 3.10, page 358.
        Q = 5000 # kg/s Emission Rate
        K = 2 # Diffusion Constant
        H = 2 # m Height
        u = 1 # m/s Wind Speed

        value = (Q / (2 * math.pi * K * -x)) * math.exp(- (u * ((pow(y, 2) + pow(H, 2))) / (4 * K * -x)))

        if value < 0:
            return 420.0
        else:
            return 420.0 + value

    def position_callback(self, data):

        ppm = self.calculateCO2(data)

        self.pub.publish(CO2(ppm=ppm,
                             average_temp=55.0,
                             humidity=0.0,
                             humidity_sensor_temp=0.0,
                             atmospheric_pressure=800,
                             detector_temp=55.0,
                             source_temp=55.0,
                             status=CO2.NO_ERROR))

    def publish(self):
        self.node.create_subscription(NavSatFix, f"{self.id}/mavros/global_position/global",
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
