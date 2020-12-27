#! /usr/bin/env python
import rospy
from ActionState import ActionState
from dragonfly_messages.msg import SemaphoreToken
from rx.subjects import Subject
from rx.core import Observable

class SemaphoreAction:

    def __init__(self, id, semaphore_id, drones):
        self.id = id
        self.semaphore_id = semaphore_id
        self.drones = set(drones)
        self.commanded = False
        self.status = ActionState.WORKING

        self.publish_semaphore_interval = Observable.empty().subscribe()
        self.receive_semaphore_subscription = None
        self.semaphore_publisher = rospy.Publisher("/dragonfly/semaphore", SemaphoreToken, queue_size=1)

        self.semaphore_subject = Subject()

        self.responded = {id}

    def publishSemaphore(self, time):
        self.semaphore_publisher.publish(SemaphoreToken(self.id, self.semaphore_id))

    def handleToken(self, token):
        if token.id == self.semaphore_id:
            self.responded.add(token.drone)

        if self.responded == self.drones:
            self.status = ActionState.SUCCESS
            self.stop()

    def printError(self, e):
        print "Error while subscibing to semaphore: {}".format(e)

    def step(self):
        if not self.commanded:
            self.commanded = True

            # Publish semaphore until finished
            self.publish_semaphore_ten_times = Observable.interval(1000).take(10).subscribe(
                on_next=self.publishSemaphore,
                on_error=lambda e: self.printError(e))
            self.publish_semaphore_interval = Observable.interval(1000).subscribe(
                on_next=self.publishSemaphore,
                on_error=lambda e: self.printError(e))

            self.receive_semaphore_subscription = rospy.Subscriber("/dragonfly/semaphore", SemaphoreToken, lambda token: self.semaphore_subject.on_next(token))
            self.semaphore_subject.subscribe(lambda token: self.handleToken(token))

        return self.status

    def stop(self):
        self.publish_semaphore_interval.dispose()
        self.semaphore_subject.dispose()
        if self.receive_semaphore_subscription is not None:
            self.receive_semaphore_subscription.unregister()