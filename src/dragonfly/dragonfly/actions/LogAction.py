#!/usr/bin/env python
from .ActionState import ActionState
from std_msgs.msg import String

class LogAction:

    def __init__(self, logPublisher, message):
        self.message = message
        self.logPublisher = logPublisher

    def step(self):
        self.logPublisher.publish(String(data=self.message))

        return ActionState.SUCCESS

    def stop(self):
        pass
