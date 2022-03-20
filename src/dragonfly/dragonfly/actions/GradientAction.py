#!/usr/bin/env python
import math
import rx
import rx.operators as ops

import numpy as np
from geometry_msgs.msg import TwistStamped
from rx.subject import Subject
from sklearn.linear_model import LinearRegression

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
    SAMPLE_RATE = .01

    def __init__(self, id, log_publisher, local_setvelocity_publisher, drones, droneStreamFactory):
        self.id = id
        self.log_publisher = log_publisher
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.drones = set(drones)
        self.commanded = False
        self.status = ActionState.WORKING
        self.droneStreamFactory = droneStreamFactory

        self.gradient_subscription = rx.empty().subscribe()
        self.timerSubscription = rx.empty().subscribe()
        self.max_value = None

    def parseReading(self, reading):
        return float(reading.data.split()[3])

    def checkForMax(self, readingPosition):
        if self.max_value is None or readingPosition.value > self.max_value.value:
            self.timerSubscription.dispose()
            self.timerSubscription = rx.timer(10) \
                .subscribe(on_next=lambda v: self.complete())
            self.max_value = readingPosition
            print("Max: {} at {} {}".format(self.max_value.value, self.max_value.latitude, self.max_value.longitude))

    def linearRegressionNormal(self, readingPositions):
        print("Calculating normal")
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

            droneReadingSubjects = [self.setupSubject(drone) for drone in self.drones]

            self.gradient_subscription = rx.combine_latest(*droneReadingSubjects).pipe(
                ops.sample(self.SAMPLE_RATE),
                ops.map(lambda readings: self.linearRegressionNormal(readings))
            ).subscribe(on_next=lambda vector: self.navigate(vector))

        return self.status

    def stop(self):
        self.gradient_subscription.dispose()
        self.navigate(dotdict({"x": 0, "y": 0}))

    def setupSubject(self, drone):

        drone_streams = self.droneStreamFactory.get_drone(drone)

        position_subject = drone_streams.get_position()
        co2_subject = drone_streams.get_co2()

        position_value_subject = Subject()

        rx.combine_latest(position_subject, co2_subject).pipe(
            ops.map(lambda tuple: ReadingPosition(tuple[0].latitude, tuple[0].longitude, self.parseReading(tuple[1])))
        ).subscribe(on_next=lambda v: position_value_subject.on_next(v))

        return position_value_subject

    def complete(self):
        self.log_publisher.publish(
            "Maximum CO2 of {} found at {}, {}".format(self.max_value.value, self.max_value.latitude,
                                                       self.max_value.longitude))
        self.status = ActionState.SUCCESS
