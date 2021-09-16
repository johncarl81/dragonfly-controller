#!/usr/bin/env python
from .ActionState import ActionState


class EmptyAction:

    def step(self):
        print("Empty Step")
        return ActionState.SUCCESS

    def stop(self):
        pass
