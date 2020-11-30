#! /usr/bin/env python
from ActionState import ActionState

class TakeoffAction:

    def __init__(self, takeoff_service, altitude):
        self.takeoff_service = takeoff_service
        self.altitude = altitude

    def step(self):
        print "Take off"
        result = self.takeoff_service(altitude = self.altitude)

        print "Take off result", result

        return ActionState.mapSuccess(result.success)

    def stop(self):
        pass