#!/usr/bin/env python3
import rx
import rx.operators as ops

from dragonfly_messages.msg import SemaphoreToken
from .ActionState import ActionState

def build_flocking_end_id(id):
    return f"flockingEnd.{id}"

class StopFlockingAction:

    def __init__(self, logger, id, semaphore_publisher):
        self.logger = logger
        self.semaphore_publisher = semaphore_publisher
        self.id = id

    def step(self):
        rx.interval(1).pipe(
            ops.take(10)
        ).subscribe(
            on_next=self.publish_semaphore,
            on_error=lambda e: self.logger.error(f"Error while publishing to semaphore: {e}"))

        return ActionState.SUCCESS

    def publish_semaphore(self, time):
        self.logger.info(f"{self.id}: Publishing flocking stop")
        token = SemaphoreToken()
        token.drone = self.id
        token.id = build_flocking_end_id(self.id)
        self.semaphore_publisher.publish(token)

    def stop(self):
        pass
