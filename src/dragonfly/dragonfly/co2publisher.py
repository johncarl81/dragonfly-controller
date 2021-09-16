#!/usr/bin/env python

import argparse

import rospy
import serial
from std_msgs.msg import String


def publishco2(id):
    rospy.loginfo("publishing co2 readings on {}/co2".format(id))
    pub = rospy.Publisher("{}/co2".format(id), String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            with serial.Serial("/dev/ttysba5", baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                               stopbits=serial.STOPBITS_ONE) as port:
                rospy.loginfo('Connected to /dev/ttysba5')
                # Publish on demand
                port.write('!')
                # Configure to 2 decimal places
                port.write('C2\r')
                while not rospy.is_shutdown():
                    port.write('M')
                    hello_str = port.readline()
                    pub.publish(hello_str)
                    rate.sleep()
        except serial.SerialException as ex:
            rospy.sleep(1)


def main():
    parser = argparse.ArgumentParser(description='Starts ROS publisher for CO2 sensor.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publishco2(args.id)


if __name__ == '__main__':
    main()
