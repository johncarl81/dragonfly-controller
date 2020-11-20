#! /usr/bin/env python
import math, rospy
from geometry_msgs.msg import PoseStamped


def distance(position1, position2):
    deltax = position1.x - position2.x
    deltay = position1.y - position2.y
    deltaz = position1.z - position2.z

    return math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))

class WaypointAction:

    def __init__(self, id, local_setposition_publisher, waypoint, waittime, distanceThreshold):
        self.commanded = False
        self.id = id
        self.waypoint = waypoint
        self.waittime = waittime
        self.distanceThreshold = distanceThreshold
        self.local_setposition_publisher = local_setposition_publisher


    def localpositionCallback(self, data):
        self.localposition = data.pose.position

    def step(self):
        if not self.commanded :
            rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped, self.localpositionCallback)
            rospy.wait_for_message("{}/mavros/local_position/pose".format(self.id), PoseStamped)
            self.local_setposition_publisher.publish(self.waypoint)

        print "Distance to point:{} {} {}".format(self.waypoint.pose.position.x, self.waypoint.pose.position.y, self.waypoint.pose.position.z), \
              distance(self.waypoint.pose.position, self.localposition)

        return distance(self.waypoint.pose.position, self.localposition) < self.distanceThreshold