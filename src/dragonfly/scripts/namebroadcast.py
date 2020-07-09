#!/usr/bin/env python

import rospy
import argparse
from std_msgs.msg import String

def publishco2(id):
    rospy.loginfo("publishing name {} on /dragonfly/broadcast".format(id))
    pub = rospy.Publisher("/dragonfly/broadcast", String,  queue_size=10)
    rospy.init_node('name_broadcast', anonymous=True)
    rate = rospy.Rate(.1)
    while True:
        pub.publish(id)
        rate.sleep()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Starts ROS publisher for name broadcast.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publishco2(args.id)
