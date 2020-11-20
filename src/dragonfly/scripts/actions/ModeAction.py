#! /usr/bin/env python

class ModeAction:

    def __init__(self, setmode_service, mode):
        self.queue = []
        self.mode = mode
        self.setmode_service = setmode_service

    def step(self):
        print "Set Mode {}".format(self.mode)
        print self.setmode_service(custom_mode = self.mode)

        return True