#! /usr/bin/env python

class ArmAction:

    def __init__(self, arm_service):
        self.arm_service = arm_service

    def step(self):
        print "Arming"
        print self.arm_service(True)

        return True