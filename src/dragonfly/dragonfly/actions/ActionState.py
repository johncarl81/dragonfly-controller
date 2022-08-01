#!/usr/bin/env python3
from enum import Enum


class ActionState(Enum):
    WORKING = 1
    SUCCESS = 2
    FAILURE = 3

    @staticmethod
    def mapSuccess(result):
        return ActionState.SUCCESS if result else ActionState.FAILURE
