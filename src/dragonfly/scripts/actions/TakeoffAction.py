#!/usr/bin/env python
from .ActionState import ActionState


class TakeoffAction:

    def __init__(self, log_publisher, takeoff_service, altitude):
        self.log_publisher = log_publisher
        self.takeoff_service = takeoff_service
        self.altitude = altitude

    def step(self):
        print("Take off")
        result = self.takeoff_service(altitude=self.altitude)

        print("Take off result {}".format(result))
        if result.success:
            self.log_publisher.publish("Takeoff to {}m".format(self.altitude))
        else:
            self.log_publisher.publish("Takeoff failed")

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
