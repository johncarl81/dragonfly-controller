#! /usr/bin/env python
from ActionState import ActionState

class ActionQueue:

    def __init__(self):
        self.queue = []

    def step(self):
        # print "Step {}".format(len(self.queue))
        if len(self.queue) > 0:
            action = self.queue[0]

            result = action.step()

            if result == ActionState.SUCCESS :
                self.queue.pop(0)
            elif result == ActionState.FAILURE:
                self.clear()
                return ActionState.FAILURE
            return ActionState.WORKING
        else:
            return ActionState.SUCCESS

    def push(self, action):
        self.queue.append(action)
        return self

    def clear(self):
        del self.queue[:]