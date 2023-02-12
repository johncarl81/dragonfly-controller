#!/usr/bin/env python3
import rx
import rx.operators as ops

from dragonfly_messages.msg import SemaphoreToken
from .ActionState import ActionState


class SemaphoreAction:

    def __init__(self, logger, id, semaphore_id, drones, semaphore_publisher, semaphore_observable):
        self.logger = logger
        self.id = id
        self.semaphore_id = semaphore_id
        self.drones = set(drones)
        self.commanded = False
        self.status = ActionState.WORKING

        self.publish_semaphore_interval = rx.empty().subscribe()
        self.receive_semaphore_subscription = rx.empty().subscribe()
        self.semaphore_publisher = semaphore_publisher
        self.semaphore_observable = semaphore_observable

        self.responded = {id}

    def publishSemaphore(self, time):
        self.logger.info(f"{self.id}: Publishing semaphore")
        token = SemaphoreToken()
        token.drone = self.id
        token.id = self.semaphore_id
        self.semaphore_publisher.publish(token)  # @TODO this is wrong

    def handleToken(self, token):
        self.logger.info(f"{self.id}: Received token")
        if token.id == self.semaphore_id:
            self.responded.add(token.drone)

        if self.responded == self.drones:
            self.status = ActionState.SUCCESS
            self.stop()
            rx.interval(1).pipe(
                ops.take(10)
            ).subscribe(
                on_next=self.publishSemaphore,
                on_error=lambda e: self.printError(e))

    def printError(self, e):
        self.logger.error(f"Error while subscribing to semaphore: {e}")

    def step(self):
        if not self.commanded:
            self.logger.info("Semaphore commanded")
            self.commanded = True

            # Publish semaphore until finished
            self.publish_semaphore_interval = rx.interval(1).subscribe(
                on_next=self.publishSemaphore,
                on_error=lambda e: self.printError(e))

            self.receive_semaphore_subscription = self.semaphore_observable.subscribe(lambda token: self.handleToken(token))

        return self.status

    def stop(self):
        self.publish_semaphore_interval.dispose()
        self.receive_semaphore_subscription.dispose()
