#! /usr/bin/env python

class DisarmAction:

    def __init__(self, arm_service):
        self.arm_service = arm_service

    def step(self):
        print "Disarming"
        print self.arm_service(False)

        return True