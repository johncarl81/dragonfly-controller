#! /usr/bin/env python
from geometry_msgs.msg import PoseStamped
from ActionState import ActionState

class SetPositionAction:

    def __init__(self, local_setposition_publisher, x, y, z):
        self.local_setposition_publisher = local_setposition_publisher
        self.x = x
        self.y = y
        self.z = z

    def step(self):
        goalPos = PoseStamped()
        goalPos.pose.position.x = self.x
        goalPos.pose.position.y = self.y
        goalPos.pose.position.z = self.z

        self.local_setposition_publisher.publish(goalPos)

        return ActionState.SUCCESS

    def stop(self):
        pass