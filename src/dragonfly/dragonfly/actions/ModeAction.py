#!/usr/bin/env python
from .ActionState import ActionState
from mavros_msgs.srv import SetMode


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
            request = SetMode.Request()
            request.custom_mode=self.mode
            future = self.setmode_service.call_async(request)
            def mode_finished():
                result = future.result()
                print("Set mode result", result)
                self.status = ActionState.mapSuccess(result.mode_sent)
            future.add_done_callback(mode_finished)
        return self.status

    def stop(self):
        pass
