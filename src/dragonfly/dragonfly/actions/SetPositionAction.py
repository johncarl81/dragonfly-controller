#!/usr/bin/env python
from ..waypointUtil import createWaypoint

from .ActionState import ActionState


class SetPositionAction:

    def __init__(self, local_setposition_publisher, x, y, z, orientation):
        self.local_setposition_publisher = local_setposition_publisher
        self.x = x
        self.y = y
        self.z = z
        self.orientation = orientation

    def step(self):
        goalPos = createWaypoint(self.x, self.y, self.z, self.orientation)

        self.local_setposition_publisher.publish(goalPos)

        return ActionState.SUCCESS

    def stop(self):
        pass
