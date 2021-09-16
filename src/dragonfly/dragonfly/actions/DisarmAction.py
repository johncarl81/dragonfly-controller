#!/usr/bin/env python
from .ActionState import ActionState


class DisarmAction:

    def __init__(self, log_publisher, arm_service):
        self.log_publisher = log_publisher
        self.arm_service = arm_service

    def step(self):
        print("Disarming")
        result = self.arm_service(False)

        print("Disarming result {}".format(result))

        if result.success:
            self.log_publisher.publish("Disarmed")
        else:
            self.log_publisher.publish("Disarm failed")

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
