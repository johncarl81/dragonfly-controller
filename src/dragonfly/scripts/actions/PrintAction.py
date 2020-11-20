#! /usr/bin/env python

class PrintAction:

    def __init__(self, message):
        self.message = message

    def step(self):
        print self.message

        return True