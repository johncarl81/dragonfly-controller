#!/usr/bin/env python
import rx

from dragonfly_messages.msg import SemaphoreToken
from .ActionState import ActionState


class SemaphoreAction:

    def __init__(self, id, semaphore_id, drones, semaphore_publisher, semaphore_observable):
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
        self.semaphore_publisher.publish(SemaphoreToken(self.id, self.semaphore_id))  # @TODO this is wrong

    def handleToken(self, token):
        if token.id == self.semaphore_id:
            self.responded.add(token.drone)

        if self.responded == self.drones:
            self.status = ActionState.SUCCESS
            self.stop()
            rx.interval(1000).take(10).subscribe(
                on_next=self.publishSemaphore,
                on_error=lambda e: self.printError(e))

    def printError(self, e):
        print("Error while subscibing to semaphore: {}".format(e))

    def step(self):
        if not self.commanded:
            self.commanded = True

            # Publish semaphore until finished
            self.publish_semaphore_interval = rx.interval(1000).subscribe(
                on_next=self.publishSemaphore,
                on_error=lambda e: self.printError(e))

            self.receive_semaphore_subscription = self.semaphore_observable.subscribe(lambda token: self.handleToken(token))

        return self.status

    def stop(self):
        self.publish_semaphore_interval.dispose()
        self.receive_semaphore_subscription.dispose()
