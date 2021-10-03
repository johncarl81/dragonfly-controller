#!/usr/bin/env python

import argparse
import math

import rospy
from sensor_msgs.msg import NavSatFix
from dragonfly_messages.msg import CO2


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class VirtualCO2Publisher:
    BALLOON_FIESTA_VIRTUAL_SOURCE = dotdict({
        "latitude": 35.13960,
        "longitude": -106.55735
    })

    ARROYO_DEL_OSO_VIRTUAL_SOURCE = dotdict({
        "latitude": 35.139607,
        "longitude": -106.55735
    })

    def __init__(self, id):
        self.id = id
        self.pub = rospy.Publisher("{}/co2".format(id), CO2, queue_size=10)

    def differenceInMeters(self, one, two):
        earthCircumference = 40008000
        return [
            ((one.longitude - two.longitude) * (earthCircumference / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (earthCircumference / 360))
        ]

    def calculateCO2(self, position):

        [y, x] = self.differenceInMeters(position, self.ARROYO_DEL_OSO_VIRTUAL_SOURCE)

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

        reading = CO2()
        reading.ppm = co2
        reading.sensor_temp = 55.0
        reading.humidity = 0.0
        reading.humidity_sensor_temp = 0.0
        reading.atmospheric_pressure = 800
        reading.detector_temp = 55.0
        reading.source_temp = 55.0
        reading.status = 0

        self.pub.publish(reading)

    def publish(self):
        rospy.Subscriber("{}/mavros/global_position/global".format(self.id), NavSatFix, self.position_callback)

    def publishco2(id):
        rospy.loginfo("publishing co2 readings on {}/co2".format(id))


if __name__ == '__main__':
    rospy.init_node('virtual_co2', anonymous=True)

    parser = argparse.ArgumentParser(description='Starts ROS publisher for CO2 sensor.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publisher = VirtualCO2Publisher(args.id)

    publisher.publish()

    rospy.spin()
