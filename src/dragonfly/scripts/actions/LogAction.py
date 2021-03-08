#!/usr/bin/env python
from .ActionState import ActionState


class LogAction:

    def __init__(self, logPublisher, message):
        self.message = message
        self.logPublisher = logPublisher

    def step(self):
        self.logPublisher.publish(self.message)

        return ActionState.SUCCESS

    def stop(self):
        pass
