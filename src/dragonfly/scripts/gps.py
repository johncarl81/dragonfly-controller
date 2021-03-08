#!/usr/bin/env python
import argparse

import rospy
from sensor_msgs.msg import NavSatFix

position = None


def callback(data):
    global position
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)


def listener(id):
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gpslistener', anonymous=True)

    rospy.Subscriber("{}/mavros/global_position/global".format(id), NavSatFix, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Listen to and print the given drone\'s GPS.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    listener(args.id)
