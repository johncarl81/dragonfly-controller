#!/usr/bin/env python
from .ActionState import ActionState


class PrintAction:

    def __init__(self, message):
        self.message = message

    def step(self):
        print(self.message)

        return ActionState.SUCCESS

    def stop(self):
        pass
