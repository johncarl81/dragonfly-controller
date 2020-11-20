#! /usr/bin/env python

class LogAction:

    def __init__(self, logPublisher, message):
        self.message = message
        self.logPublisher = logPublisher

    def step(self):
        self.logPublisher.publish(self.message)

        return True