#!/usr/bin/env python
import rospy
import serial
import argparse
from std_msgs.msg import String

def publishco2(id):
    print "publishing co2 readings on {}/co2".format(id)
    pub = rospy.Publisher("{}/co2".format(id), String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    with serial.Serial("/dev/ttysba5", baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE) as port:
        # Publish on demand
        port.write('!')
        while True:
            port.write('M')
            hello_str = port.readline()
            rospy.loginfo(hello_str)
            pub.publish(hello_str)
            rate.sleep()



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Starts ROS publisher for CO2 sensor.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publishco2(args.id)
