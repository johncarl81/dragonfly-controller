#!/usr/bin/env python
import rx

from std_msgs.msg import String
from .ActionState import ActionState

class WaitForDisarmAction:

    def __init__(self, id, log_publisher, status_observable):
        self.status_observable = status_observable
        self.id = id
        self.log_publisher = log_publisher
        self.status = ActionState.WORKING
        self.commanded = False
        self.status_subscription = rx.empty().subscribe()

    def step(self):
        if not self.commanded:
            print("Waiting for disarm...")
            self.commanded = True

            def updateState(state):

                if not state.armed:
                    self.status = ActionState.SUCCESS
                    self.stop()
                    self.log_publisher.publish(String(data="Disarmed"))

            self.status_subscription = self.status_observable.subscribe(
                on_next = lambda state: updateState(state)
            )

        return self.status

    def stop(self):
        self.status_subscription.dispose()
