#!/usr/bin/env python3
from .ActionState import ActionState


class EmptyAction:

    def __init__(self, logger):
        self.logger = logger

    def step(self):
        self.logger.info("Empty Step")
        return ActionState.SUCCESS

    def stop(self):
        pass
