#!/usr/bin/env python3
from .ActionState import ActionState
from std_msgs.msg import String

class WaitForZeroAction:

    def __init__(self, log_publisher, zeroingSingleton):
        self.log_publisher = log_publisher
        self.logged = False
        self.zeroingSingleton = zeroingSingleton

    def step(self):
        if self.zeroingSingleton.zeroing:
            if not self.logged:
                self.logged = True
                self.log_publisher.publish(String(data="Waiting for zero..."))
            return ActionState.WORKING
        else:
            return ActionState.SUCCESS

    def stop(self):
        pass
