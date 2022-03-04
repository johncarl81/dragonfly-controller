#!/usr/bin/env python
from mavros_msgs.srv import CommandTOL

from .ActionState import ActionState
from std_msgs.msg import String

class TakeoffAction:

    def __init__(self, log_publisher, takeoff_service, altitude):
        self.log_publisher = log_publisher
        self.takeoff_service = takeoff_service
        self.altitude = altitude

    def step(self):
        print("Take off")
        result = self.takeoff_service.call(CommandTOL.Request(altitude=self.altitude))

        print("Take off result {}".format(result))
        if result and result.success:
            self.log_publisher.publish(String(data="Takeoff to {}m".format(self.altitude)))
        else:
            self.log_publisher.publish(String(data="Takeoff failed"))

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
