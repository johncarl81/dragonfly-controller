#!/usr/bin/env python
from mavros_msgs.srv import CommandBool
from .ActionState import ActionState
from std_msgs.msg import String


class DisarmAction:

    def __init__(self, log_publisher, arm_service):
        self.log_publisher = log_publisher
        self.arm_service = arm_service

    def step(self):
        print("Disarming")
        result = self.arm_service.call(CommandBool.Request(value=False))

        print("Disarming result {}".format(result))

        if result.success:
            self.log_publisher.publish(String(data="Disarmed"))
        else:
            self.log_publisher.publish(String(data="Disarm failed"))

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
