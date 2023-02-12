#!/usr/bin/env python3
from mavros_msgs.srv import CommandBool
from .ActionState import ActionState
from std_msgs.msg import String

class ArmAction:

    def __init__(self, logger, log_publisher, arm_service):
        self.logger = logger
        self.log_publisher = log_publisher
        self.arm_service = arm_service

    def step(self):
        self.logger.info("Arming")
        result = self.arm_service.call(CommandBool.Request(value=True))
        self.logger.info(f"Arming result {result}")
        if result and result.success:
            self.log_publisher.publish(String(data="Armed"))
        else:
            self.log_publisher.publish(String(data="Arming failed"))

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
