#! /usr/bin/env python
from ActionState import ActionState

class MissionStartAction:

    def __init__(self, mission_starter):
        self.mission_starter = mission_starter

    def step(self):
        if self.mission_starter.start:
            self.mission_starter.start = False
            return ActionState.SUCCESS
        return ActionState.WORKING

    def stop(self):
        pass