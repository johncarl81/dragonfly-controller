#! /usr/bin/env python

class ActionQueue:

    def __init__(self):
        self.queue = []

    def step(self):
        # print "Step {}".format(len(self.queue))
        if len(self.queue) > 0:
            print "Trying 0"
            action = self.queue[0]

            if action.step():
                self.queue.pop(0)
            return True
        return False

    def push(self, action):
        self.queue.append(action)
        return self