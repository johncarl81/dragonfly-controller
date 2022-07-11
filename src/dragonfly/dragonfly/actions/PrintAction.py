#!/usr/bin/env python3
from .ActionState import ActionState


class PrintAction:

    def __init__(self, message):
        self.message = message

    def step(self):
        print(self.message)

        return ActionState.SUCCESS

    def stop(self):
        pass
