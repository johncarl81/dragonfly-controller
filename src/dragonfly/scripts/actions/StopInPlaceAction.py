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
        self.position_update = None

    def step(self):
        if not self.commanded:
            self.commanded = True
            def updatePosition(localposition):

                print "Stop in place"

                self.local_setposition_publisher.publish(localposition)
                self.status = ActionState.SUCCESS

                if not self.position_update is None:
                    self.position_update.unregister()

            self.position_update = rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped, updatePosition)

        return self.status


    def stop(self):
        if not self.position_update is None:
            self.position_update.unregister()