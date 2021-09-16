#!/usr/bin/env python
from mavros_msgs.msg import State

from .ActionState import ActionState


class WaitForDisarmAction:

    def __init__(self, id, log_publisher):
        self.id = id
        self.log_publisher = log_publisher
        self.status = ActionState.WORKING
        self.commanded = False
        self.disabled_update = None

    def step(self):
        if not self.commanded:
            self.commanded = True

            def updateState(state):

                if not state.armed:
                    self.status = ActionState.SUCCESS
                    self.stop()
                    self.log_publisher.publish("Disarmed")

            self.disabled_update = self.node.create_subscription(State, "{}/mavros/state".format(self.id), updateState, 10)

        return self.status

    def stop(self):
        if self.disabled_update is not None:
            self.disabled_update.destroy()
