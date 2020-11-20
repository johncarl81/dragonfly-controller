#! /usr/bin/env python
import time

class SleepAction:

    def __init__(self, duration):
        self.duration = duration

    def step(self):
        time.sleep(self.duration)

        return True