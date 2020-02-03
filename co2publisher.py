#!/usr/bin/env python
import rospy
import serial
import argparse
from std_msgs.msg import String

def publishco2(id):

    pub = rospy.Publisher("{}/co2".format(id), String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    with serial.Serial("/dev/ttysba5", baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE) as port:
        while True:
            hello_str = port.readline()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            # rate.sleep()



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Starts ROS publisher for CO2 sensor.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publishco2(id)
