#!/usr/bin/env python3
import rx
import rx.operators as ops

from .ActionQueue import ActionQueue
from .ActionState import ActionState
from std_msgs.msg import String


class ArmedStateAction:

    def __init__(self, logger, log_publisher, id, status_observable):
        self.logger = logger
        self.status_observable = status_observable
        self.log_publisher = log_publisher
        self.id = id
        self.armedQueue = ActionQueue()
        self.notarmedQueue = ActionQueue()
        self.status = ActionState.WORKING
        self.commanded = False
        self.status_subscription = rx.empty().subscribe()

    def step(self):
        if not self.commanded:
            self.logger.info("Verifying disarmed...")
            self.commanded = True

            def updateState(state):

                if state.armed:
                    self.logger.info("Is already armed, failed")
                    self.status = ActionState.FAILURE
                    self.log_publisher.publish(String(data="Arming failed, already armed"))
                else:
                    self.logger.info("Is not armed, continue")
                    self.status = ActionState.SUCCESS

            self.status_subscription = self.status_observable.pipe(
                ops.take(1)
            ).subscribe(
                on_next = lambda state:updateState(state)
            )

        return self.status

    def stop(self):
        self.status_subscription.dispose()
