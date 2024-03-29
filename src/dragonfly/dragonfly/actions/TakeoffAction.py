#!/usr/bin/env python3
from mavros_msgs.srv import CommandTOL

from .ActionState import ActionState
from std_msgs.msg import String

class TakeoffAction:

    def __init__(self, logger, log_publisher, takeoff_service, altitude):
        self.logger = logger
        self.log_publisher = log_publisher
        self.takeoff_service = takeoff_service
        self.altitude = altitude

    def step(self):
        self.logger.info("Take off")
        result = self.takeoff_service.call(CommandTOL.Request(altitude=self.altitude))

        self.logger.info(f"Take off result: {result}")
        if result and result.success:
            self.log_publisher.publish(String(data=f"Takeoff to {self.altitude}m"))
        else:
            self.log_publisher.publish(String(data="Takeoff failed"))

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
