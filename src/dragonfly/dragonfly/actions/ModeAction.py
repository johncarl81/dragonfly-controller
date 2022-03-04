#!/usr/bin/env python
from mavros_msgs.srv import SetMode

from .ActionState import ActionState

class ModeAction:

    def __init__(self, setmode_service, mode):
        self.queue = []
        self.mode = mode
        self.setmode_service = setmode_service
        self.commanded = False
        self.status = ActionState.WORKING

    def step(self):
        if not self.commanded:
            self.commanded = True
            print("Set Mode {}".format(self.mode))
            future = self.setmode_service.call_async(SetMode.Request(custom_mode=self.mode))

            def mode_finished(msg):
                result = future.result()
                print("Set mode result", result, msg)
                self.status = ActionState.mapSuccess(result.mode_sent)

            future.add_done_callback(mode_finished)
        return self.status

    def stop(self):
        pass
