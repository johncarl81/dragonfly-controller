#!/usr/bin/env python
from .ActionState import ActionState


class LandAction:

    def __init__(self, log_publisher, land_service):
        self.log_publisher = log_publisher
        self.land_service = land_service

    def step(self):
        print("Land off")
        result = self.land_service(altitude=0)

        print("Land result {}".format(result))

        if result.success:
            self.log_publisher.publish("Landing...")
        else:
            self.log_publisher.publish("Landing failed")

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass
