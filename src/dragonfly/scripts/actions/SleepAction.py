#!/usr/bin/env python
import time

from .ActionState import ActionState


class SleepAction:

    def __init__(self, duration):
        self.duration = duration
        self.start = None

    def step(self):
        if self.start == None:
            self.start = time.time()
            print("Sleeping for {}".format(self.duration))

        if time.time() - self.start > self.duration:
            return ActionState.SUCCESS
        else:
            return ActionState.WORKING

    def stop(self):
        pass
