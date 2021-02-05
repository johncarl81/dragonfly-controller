#! /usr/bin/env python
import math, rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from ActionState import ActionState

MAX_SPEED = 2.5
SLOWDOWN_SPEED = 0.8
SLOWDOWN_THRESHOLD = 3

def distance(position1, position2):
    deltax = position1.x - position2.x
    deltay = position1.y - position2.y
    deltaz = position1.z - position2.z

    return math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))

def difference(position1, position2):

    twist = TwistStamped()

    dx = position2.pose.position.x - position1.pose.position.x
    dy = position2.pose.position.y - position1.pose.position.y
    dz = position2.pose.position.z - position1.pose.position.z

    magnitude = math.sqrt((dx * dx) + (dy * dy) + (dz * dz))

    if magnitude > SLOWDOWN_THRESHOLD :
        twist.twist.linear.x = dx * MAX_SPEED / magnitude
        twist.twist.linear.y = dy * MAX_SPEED / magnitude
        twist.twist.linear.z = dz * MAX_SPEED / magnitude
    else:
        twist.twist.linear.x = dx * SLOWDOWN_SPEED / magnitude
        twist.twist.linear.y = dy * SLOWDOWN_SPEED / magnitude
        twist.twist.linear.z = dz * SLOWDOWN_SPEED / magnitude

    return twist

class WaypointDirectAction:

    def __init__(self, id, local_setvelocity_publisher, waypoint, distanceThreshold):
        self.id = id
        self.waypoint = waypoint
        self.distanceThreshold = distanceThreshold
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.status = ActionState.WORKING
        self.commanded = False
        self.position_update = None

    def step(self):
        if not self.commanded:
            self.commanded = True
            def updatePosition(localposition):

                # print "Distance to point:{} {} {}".format(self.waypoint.pose.position.x, self.waypoint.pose.position.y, self.waypoint.pose.position.z), \
                #       distance(self.waypoint.pose.position, localposition.pose.position)

                unit_vector = difference(localposition, self.waypoint)

                self.local_setvelocity_publisher.publish(unit_vector)

                if distance(self.waypoint.pose.position, localposition.pose.position) < self.distanceThreshold:
                    self.status = ActionState.SUCCESS
                    zero_vector = TwistStamped()
                    zero_vector.twist.linear.x = 0
                    zero_vector.twist.linear.y = 0
                    zero_vector.twist.linear.z = 0
                    self.local_setvelocity_publisher.publish(zero_vector)
                    self.stop()

            self.position_update = rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped, updatePosition)

        return self.status

    def stop(self):
        if self.position_update is not None:
            self.position_update.unregister()
            self.position_update = None
