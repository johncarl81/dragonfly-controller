#! /usr/bin/env python
import math, rospy
from geometry_msgs.msg import PoseStamped
from waypointUtil import *
from ActionState import ActionState


class StopInPlaceAction:

    def __init__(self, id, local_setposition_publisher):
        self.id = id
        self.local_setposition_publisher = local_setposition_publisher
        self.status = ActionState.WORKING
        self.commanded = False

    def step(self):
        if not self.commanded:
            self.commanded = True
            def updatePosition(localposition):

                print "Set mode result", result

                self.local_setposition_publisher.publish(localposition)
                self.status = ActionState.SUCCESS
                position_update.unregister()

            position_update = rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped, updatePosition)

        return self.status
