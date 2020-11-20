#! /usr/bin/env python

class TakeoffAction:

    def __init__(self, takeoff_service, altitude):
        self.takeoff_service = takeoff_service
        self.altitude = altitude

    def step(self):
        print "Take off"
        print self.takeoff_service(altitude = self.altitude)

        return True