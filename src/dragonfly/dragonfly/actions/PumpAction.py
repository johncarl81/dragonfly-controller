#!/usr/bin/env python3
from dragonfly_messages.srv import Pump
from .ActionState import ActionState

class PumpAction:

    def __init__(self, pump_num, pump_service):
        self.pump_num = pump_num
        self.pump_service = pump_service

    def step(self):
        self.pump_service.call(Pump.Request(pump_num=self.pump_num))
        return ActionState.SUCCESS

    def stop(self):
        pass
