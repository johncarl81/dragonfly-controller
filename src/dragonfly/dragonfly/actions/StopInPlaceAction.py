#!/usr/bin/env python
import rx
import rx.operators as ops

from .ActionState import ActionState
from std_msgs.msg import String

class StopInPlaceAction:

    def __init__(self, id, log_publisher, local_setposition_publisher, local_position_observable):
        self.id = id
        self.log_publisher = log_publisher
        self.local_setposition_publisher = local_setposition_publisher
        self.status = ActionState.WORKING
        self.commanded = False
        self.local_position_observable = local_position_observable
        self.position_update = rx.empty().subscribe()

    def step(self):
        if not self.commanded:
            self.commanded = True

            def updatePosition(localposition):
                self.local_setposition_publisher.publish(localposition)
                self.status = ActionState.SUCCESS

                self.stop()

                print("Stop in place")
                self.log_publisher.publish(String(data="Stopped"))

            self.position_update = self.local_position_observable.pipe(
                ops.take(1)
            ).subscribe(
                on_next = lambda position: updatePosition(position)
            )

        return self.status

    def stop(self):
        self.position_update.dispose()
