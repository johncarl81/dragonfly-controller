#!/usr/bin/env python
import rospy
import serial
from std_msgs.msg import String

pub = rospy.Publisher('JUAV1/co2', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10) # 10hz
with serial.Serial("/dev/ttyUSB0", baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE) as port:
    while True:
        hello_str = port.readline()
        #rospy.loginfo(hello_str)
        pub.publish(hello_str)
        # rate.sleep()


