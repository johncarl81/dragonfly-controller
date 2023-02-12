#!/usr/bin/env python3
import time

from .ActionState import ActionState


class SleepAction:

    def __init__(self, logger, duration):
        self.logger = logger
        self.duration = duration
        self.start = None

    def step(self):
        if self.start is None:
            self.start = time.time()
            self.logger.info(f"Sleeping for {self.duration}")

        if time.time() - self.start > self.duration:
            return ActionState.SUCCESS
        else:
            return ActionState.WORKING

    def stop(self):
        pass
