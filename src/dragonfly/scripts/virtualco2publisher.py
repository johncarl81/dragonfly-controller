#!/usr/bin/env python

import rospy
import argparse
import math
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

class VirtualCO2Publisher:

    VIRTUAL_SOURCE = dotdict({
        "latitude": 35.195058873,
        "longitude": -106.5960480158447
    })

    def __init__(self, id):
        self.id = id
        self.pub = rospy.Publisher("{}/co2".format(id), String, queue_size=10)

    def differenceInMeters(self, one, two):
        earthCircumference = 40008000
        return [
            ((one.longitude - two.longitude) * (earthCircumference / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (earthCircumference / 360))
        ]

    def calculateCO2(self, position):


        [y, x] = self.differenceInMeters(position, self.VIRTUAL_SOURCE)

        if x == 0:
            x = 0.00001

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

        self.pub.publish("M 55146 52516 {} 55.0 0.0 0.0 800 55.0 55.0 00".format(co2))

    def publish(self):
        rospy.Subscriber("{}/mavros/global_position/global".format(self.id), NavSatFix, self.position_callback)

    def publishco2(id):
        rospy.loginfo("publishing co2 readings on {}/co2".format(id))

if __name__ == '__main__':
    rospy.init_node('virtual_co2', anonymous=True)

    parser = argparse.ArgumentParser(description = 'Starts ROS publisher for CO2 sensor.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publisher = VirtualCO2Publisher(args.id)

    publisher.publish()

    rospy.spin()
