#!/usr/bin/env python
import math
import rx
import rx.operators as ops

from geometry_msgs.msg import TwistStamped
from rx.subject import Subject
from std_msgs.msg import String

from .ActionState import ActionState


class FlockingAction:
    SAMPLE_RATE = 0.1
    DAMPENING = 0.6
    REPULSION_COEFFIENT = 4.0
    REPULSION_RADIUS = 4.0
    POSITION_ATTRACTION = 2.0
    POSITION_ATTRACTION_RADIUS = 3.0

    def __init__(self, id, log_publisher, local_setvelocity_publisher, announce_stream, xoffset, yoffset, leader, drone_stream_factory):
        self.log_publisher = log_publisher
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.id = id
        self.xoffset = xoffset
        self.yoffset = yoffset
        self.leader = leader
        self.started = False
        self.ros_subscriptions = []
        self.announce_stream = announce_stream
        self.drone_stream_factory = drone_stream_factory

        self.flock_coordinates = {}

        self.flock_repulsion = Subject()

        self.zero_velocity_debounce_subscription = rx.empty().subscribe()
        self.flocking_subscription = rx.empty().subscribe()
        self.flockSubscription = rx.empty().subscribe()
        self.navigate_subscription = rx.empty().subscribe()

    def navigate(self, input):

        twist = TwistStamped()

        for vector in input:
            twist.twist.linear.x += vector[0]
            twist.twist.linear.y += vector[1]

        self.local_setvelocity_publisher.publish(twist)

    def differenceInMeters(self, one, two):
        earthCircumference = 40008000
        return [
            ((one.longitude - two.longitude) * (earthCircumference / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (earthCircumference / 360))
        ]

    def formation_position(self, leaderPosition, selfPosition):
        difference = self.differenceInMeters(leaderPosition, selfPosition)
        difference[0] += self.xoffset
        difference[1] += self.yoffset
        difference_magnitude = self.magnitude(difference)

        return [
            self.POSITION_ATTRACTION * difference[0] / max(self.POSITION_ATTRACTION_RADIUS, difference_magnitude),
            self.POSITION_ATTRACTION * difference[1] / max(self.POSITION_ATTRACTION_RADIUS, difference_magnitude)
        ]

    def velocity_dampening(self, leadertwist, selftwist):
        return [
            self.DAMPENING * (leadertwist.twist.linear.x - selftwist.twist.linear.x),
            self.DAMPENING * (leadertwist.twist.linear.y - selftwist.twist.linear.y)
        ]

    def format_velocities(self, twist):
        return [
            twist.twist.linear.x,
            twist.twist.linear.y
        ]

    def magnitude(self, vector):
        return math.sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]))

    def repulsion_vector(self, positions):
        vector = [0, 0]
        if len(positions) > 1:
            self_position = positions[0]
            for position in positions[1:]:
                difference = self.differenceInMeters(self_position, position)
                difference_magnitude = self.magnitude(difference)
                if difference_magnitude < self.REPULSION_RADIUS:
                    vector[0] += self.REPULSION_COEFFIENT * (self.REPULSION_RADIUS - difference_magnitude) * difference[
                        0] / difference_magnitude
                    vector[1] += self.REPULSION_COEFFIENT * (self.REPULSION_RADIUS - difference_magnitude) * difference[
                        1] / difference_magnitude

        return vector

    def subscribe_flock(self):
        flock_coordinate_subject_list = [self.drone_stream_factory.get_drone(self.id).get_position()]
        flock_coordinate_subject_list.extend(self.flock_coordinates.values())

        self.flockSubscription = rx.combine_latest(*flock_coordinate_subject_list).pipe(
            ops.map(lambda positions: self.repulsion_vector(positions)),
            ops.sample(self.SAMPLE_RATE)
        ).subscribe(on_next=lambda v: self.flock_repulsion.on_next(v))

    def flock_announce(self, name):
        if name != self.id and name not in self.flock_coordinates:
            self.log_publisher.publish(String(data="Flocking with {}".format(name)))
            print("Registering flock member: {}".format(name))

            flock_coordinate_subject = self.drone_stream_factory.get_drone(name).get_position()

            self.flock_coordinates[name] = flock_coordinate_subject

            timeoutSubscription = None

            def coordinate_subscription_timeout():
                del self.flock_coordinates[name]
                self.subscribe_flock()
                timeoutSubscription.dispose()

            timeoutSubscription = flock_coordinate_subject.pipe(
                ops.debounce(10)
            ).subscribe(on_next=lambda v: coordinate_subscription_timeout())

            self.flockSubscription.dispose()

            self.subscribe_flock()

    def step(self):
        if not self.started:
            self.started = True

            print("Subscribing...")

            leader_drone = self.drone_stream_factory.get_drone(self.leader)
            self_drone = self.drone_stream_factory.get_drone(self.id)
            leaderposition_subject = leader_drone.get_position()
            selfposition_subject = self_drone.get_position()
            leadervelocity_subject = leader_drone.get_velocity()
            selfvelocity_subject = self_drone.get_velocity()

            formation_position_attraction = rx.combine_latest(leaderposition_subject, selfposition_subject).pipe(
                ops.map(lambda positions: self.formation_position(positions[0], positions[1])))

            # Issue a zero velocity token if nothing has been given for a while
            self.zero_velocity_debounce_subscription = leadervelocity_subject.pipe(
                ops.debounce(1)
            ).subscribe(on_next=lambda v: leadervelocity_subject.on_next(TwistStamped()))

            leaderVelocity = leadervelocity_subject.pipe(
                ops.map(lambda twist: self.format_velocities(twist)))

            leaderVelocityDampening = rx.combine_latest(leadervelocity_subject, selfvelocity_subject).pipe(
                ops.map(lambda twists: self.velocity_dampening(twists[0], twists[1])))

            self.navigate_subscription = rx.combine_latest(
                leaderVelocity, leaderVelocityDampening, formation_position_attraction, self.flock_repulsion).pipe(
                ops.sample(self.SAMPLE_RATE)
            ).subscribe(lambda vectors: self.navigate(vectors))

            self.flocking_subscription = self.announce_stream.subscribe(
                on_next = lambda name: self.flock_announce(name.data)
            )

            self.log_publisher.publish(String(data="Flocking"))

        return ActionState.WORKING

    def stop(self):
        self.zero_velocity_debounce_subscription.dispose()
        self.flocking_subscription.dispose()
        self.navigate_subscription.dispose()
        self.flockSubscription.dispose()
