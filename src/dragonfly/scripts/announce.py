#!/usr/bin/env python

import argparse

import rospy
from std_msgs.msg import String


def publishco2(id):
    rospy.loginfo("publishing name {} on /dragonfly/announce".format(id))
    pub = rospy.Publisher("/dragonfly/announce", String,  queue_size=10)
    rospy.init_node('name_announcement', anonymous=True)
    rate = rospy.Rate(.1)
    while not rospy.is_shutdown():
        pub.publish(id)
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Starts ROS publisher for name announcement.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publishco2(args.id)
