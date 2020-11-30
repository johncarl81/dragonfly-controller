#! /usr/bin/env python
from ActionState import ActionState

class ArmAction:

    def __init__(self, arm_service):
        self.arm_service = arm_service

    def step(self):
        print "Arming"
        result = self.arm_service(True)

        print "Arming result", result

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass