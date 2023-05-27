#!/usr/bin/env python3
from .ActionState import ActionState


class PrintAction:

    def __init__(self, logger, message):
        self.logger = logger
        self.message = message

    def step(self):
        self.logger.info(self.message)

        return ActionState.SUCCESS

    def stop(self):
        pass
