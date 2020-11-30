#! /usr/bin/env python
from ActionState import ActionState

class DisarmAction:

    def __init__(self, arm_service):
        self.arm_service = arm_service

    def step(self):
        print "Disarming"
        result = self.arm_service(False)

        print "Disarming result", result

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass