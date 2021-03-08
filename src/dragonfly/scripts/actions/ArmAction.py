#!/usr/bin/env python
from .ActionState import ActionState


class ArmAction:

    def __init__(self, log_publisher, arm_service):
        self.log_publisher = log_publisher
        self.arm_service = arm_service

    def step(self):
        print("Arming")
        result = self.arm_service(True)

        print("Arming result {}".format(result))

        if result.success:
            self.log_publisher.publish("Armed")
        else:
            self.log_publisher.publish("Arming failed")

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
