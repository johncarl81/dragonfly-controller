#!/usr/bin/env python
import math

import numpy as np
from geometry_msgs.msg import TwistStamped
from rx.core import Observable
from rx.subject import Subject
from sensor_msgs.msg import NavSatFix
from sklearn.linear_model import LinearRegression
from std_msgs.msg import String

from .ActionState import ActionState


class dotdict(dict):
    """dot.notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


class ReadingPosition:

    def __init__(self, latitude, longitude, value):
        self.latitude = latitude
        self.longitude = longitude
        self.value = value


class GradientAction:
    MAX_VELOCITY = 1.0
    SAMPLE_RATE = 10

    def __init__(self, id, log_publisher, local_setvelocity_publisher, drones, node):
        self.id = id
        self.log_publisher = log_publisher
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.drones = set(drones)
        self.commanded = False
        self.status = ActionState.WORKING
        self.node = node

        self.ros_subscriptions = []
        self.gradient_subscription = Observable.empty().subscribe()
        self.timerSubscription = Observable.empty().subscribe()
        self.max_value = None

    def parseReading(self, reading):
        return float(reading.data.split()[3])

    def checkForMax(self, readingPosition):
        if self.max_value is None or readingPosition.value > self.max_value.value:
            self.timerSubscription.dispose()
            self.timerSubscription = Observable.timer(10000) \
                .subscribe(on_next=lambda v: self.complete())
            self.max_value = readingPosition
            print("Max: {} at {} {}".format(self.max_value.value, self.max_value.latitude, self.max_value.longitude))

    def linearRegressionNormal(self, readingPositions):

        x = []
        y = []

        for readingPosition in readingPositions:
            x.append([readingPosition.longitude, readingPosition.latitude])
            y.append(readingPosition.value)
            self.checkForMax(readingPosition)

        x, y = np.array(x), np.array(y)

        model = LinearRegression().fit(x, y)

        return dotdict({
            "x": model.coef_[0],
            "y": model.coef_[1]
        })

    def navigate(self, vector):
        twist = TwistStamped()
        if vector.x != 0 and vector.y != 0:
            magnitude = math.sqrt(vector.x * vector.x + vector.y * vector.y)

            twist.twist.linear.x = self.MAX_VELOCITY * vector.x / magnitude
            twist.twist.linear.y = self.MAX_VELOCITY * vector.y / magnitude

        self.local_setvelocity_publisher.publish(twist)

    def step(self):
        if not self.commanded:
            print("Following Gradient")
            self.commanded = True

            droneReadingSubjects = []

            for drone in self.drones:
                droneReadingSubjects.append(self.setupSubject(drone))

            self.gradient_subscription = Observable.combine_latest(droneReadingSubjects,
                                                                   lambda *positionReadings: positionReadings) \
                .sample(self.SAMPLE_RATE) \
                .map(lambda readings: self.linearRegressionNormal(readings)) \
                .subscribe(on_next=lambda vector: self.navigate(vector))

        return self.status

    def stop(self):
        for subscription in self.ros_subscriptions:
            subscription.destroy()

        del self.ros_subscriptions

        self.gradient_subscription.dispose()

        self.navigate(dotdict({"x": 0, "y": 0}))

    def setupSubject(self, drone):
        position_subject = Subject()
        co2_subject = Subject()
        self.ros_subscriptions.append(
            self.node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(drone),
                                          lambda position: position_subject.on_next(position), 10))
        self.ros_subscriptions.append(
            self.node.create_subscription(String, "{}/co2".format(drone), lambda value: co2_subject.on_next(value), 10))

        position_value_subject = Subject()

        Observable.combine_latest(position_subject, co2_subject,
                                  lambda position, reading: ReadingPosition(position.latitude, position.longitude,
                                                                            self.parseReading(reading))) \
            .subscribe(on_next=lambda v: position_value_subject.on_next(v))

        return position_value_subject

    def complete(self):
        self.log_publisher.publish(
            "Maximum CO2 of {} found at {}, {}".format(self.max_value.value, self.max_value.latitude,
                                                       self.max_value.longitude))
        self.status = ActionState.SUCCESS
