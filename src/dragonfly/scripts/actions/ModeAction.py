#! /usr/bin/env python
from ActionState import ActionState

class ModeAction:

    def __init__(self, setmode_service, mode):
        self.queue = []
        self.mode = mode
        self.setmode_service = setmode_service

    def step(self):
        print "Set Mode {}".format(self.mode)
        result = self.setmode_service(custom_mode = self.mode)

        print "Set mode result", result

        return ActionState.mapSuccess(result.mode_sent)