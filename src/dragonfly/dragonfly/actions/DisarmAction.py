#!/usr/bin/env python3
from mavros_msgs.srv import CommandBool
from .ActionState import ActionState
from std_msgs.msg import String


class DisarmAction:

    def __init__(self, logger, log_publisher, arm_service):
        self.logger = logger
        self.log_publisher = log_publisher
        self.arm_service = arm_service

    def step(self):
        self.logger.info("Disarming")
        result = self.arm_service.call(CommandBool.Request(value=False))

        self.logger.info(f"Disarming result {result}")

        if result.success:
            self.log_publisher.publish(String(data="Disarmed"))
        else:
            self.log_publisher.publish(String(data="Disarm failed"))

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
