#! /usr/bin/env python
from ActionState import ActionState

class LandAction:

    def __init__(self, land_service):
        self.land_service = land_service

    def step(self):
        print "Land off"
        result = self.land_service(altitude = 0)

        print "Land result", result

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass