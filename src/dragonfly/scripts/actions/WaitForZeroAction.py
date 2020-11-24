#! /usr/bin/env python
from ActionState import ActionState

class WaitForZeroAction:

    def __init__(self, zeroingSingleton):
        self.zeroingSingleton = zeroingSingleton

    def step(self):
        if self.zeroingSingleton.zeroing:
            return ActionState.WORKING
        else:
            return ActionState.SUCCESS