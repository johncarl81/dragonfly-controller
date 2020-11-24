#! /usr/bin/env python
import math, rospy
from geometry_msgs.msg import PoseStamped
from ActionState import ActionState


def distance(position1, position2):
    deltax = position1.x - position2.x
    deltay = position1.y - position2.y
    deltaz = position1.z - position2.z

    return math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))

class WaypointAction:

    def __init__(self, id, local_setposition_publisher, waypoint, distanceThreshold):
        self.id = id
        self.waypoint = waypoint
        self.distanceThreshold = distanceThreshold
        self.local_setposition_publisher = local_setposition_publisher
        self.status = ActionState.WORKING
        self.commanded = False

    def step(self):
        if not self.commanded:
            self.commanded = True
            def updatePosition(localposition):

                # print "Distance to point:{} {} {}".format(self.waypoint.pose.position.x, self.waypoint.pose.position.y, self.waypoint.pose.position.z), \
                #       distance(self.waypoint.pose.position, localposition.pose.position)

                if distance(self.waypoint.pose.position, localposition.pose.position) < self.distanceThreshold:
                    self.status = ActionState.SUCCESS
                    position_update.unregister()

            position_update = rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped, updatePosition)

            self.local_setposition_publisher.publish(self.waypoint)

        return self.status