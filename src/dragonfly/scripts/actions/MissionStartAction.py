#!/usr/bin/env python
from .ActionState import ActionState


class MissionStartAction:

    def __init__(self, log_publisher, mission_starter):
        self.log_publisher = log_publisher
        self.mission_starter = mission_starter

    def step(self):
        if self.mission_starter.start:
            self.log_publisher.publish("Mission started")
            self.mission_starter.start = False
            return ActionState.SUCCESS
        return ActionState.WORKING

    def stop(self):
        pass
