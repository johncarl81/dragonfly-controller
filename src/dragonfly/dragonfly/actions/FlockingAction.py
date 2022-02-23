#!/usr/bin/env python
import math

from geometry_msgs.msg import TwistStamped
from rx.core import Observable
from rx.subject import Subject
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from .ActionState import ActionState


class FlockingAction:
    SAMPLE_RATE = 100.0
    DAMPENING = 0.6
    REPULSION_COEFFIENT = 4.0
    REPULSION_RADIUS = 4.0
    POSITION_ATTRACTION = 2.0
    POSITION_ATTRACTION_RADIUS = 3.0

    def __init__(self, id, log_publisher, local_setvelocity_publisher, xoffset, yoffset, leader, node):
        self.log_publisher = log_publisher
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.id = id
        self.xoffset = xoffset
        self.yoffset = yoffset
        self.leader = leader
        self.started = False
        self.ros_subscriptions = []
        self.node = node

        self.flock_coordinates = {}
        self.leaderposition_subject = Subject()
        self.selfposition_subject = Subject()
        self.leadervelocity_subject = Subject()
        self.selfvelocity_subject = Subject()
        self.flock_repulsion = Subject()

        formation_position_attraction = Observable.combine_latest(self.leaderposition_subject,
                                                                  self.selfposition_subject,
                                                                  lambda leaderposition,
                                                                         selfposition: self.formation_position(
                                                                      leaderposition, selfposition))

        # Issue a zero velocity token if nothing has been given for a while
        self.leadervelocity_subject.debounce(1000) \
            .subscribe(on_next=lambda v: self.leadervelocity_subject.on_next(TwistStamped()))

        leaderVelocity = self.leadervelocity_subject \
            .map(lambda twist: self.format_velocities(twist))

        leaderVelocityDampening = Observable.combine_latest(self.leadervelocity_subject, self.selfvelocity_subject,
                                                            lambda leadertwist, selftwist: self.velocity_dampening(
                                                                leadertwist, selftwist))

        self.navigate_subscription = Observable.combine_latest(
            [leaderVelocity, leaderVelocityDampening, formation_position_attraction, self.flock_repulsion],
            lambda *positions: positions) \
            .sample(self.SAMPLE_RATE) \
            .subscribe(lambda vectors: self.navigate(vectors))

        self.flockSubscription = Observable.empty().subscribe()

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

    def flock_announce_callback(self, nameString):
        self.flock_announce(nameString.data)

    def subscribe_flock(self):
        flock_coordinate_subject_list = [self.selfposition_subject]
        flock_coordinate_subject_list.extend(self.flock_coordinates.values())

        self.flockSubscription = Observable.combine_latest(flock_coordinate_subject_list,
                                                           lambda *positions: self.repulsion_vector(positions)) \
            .sample(self.SAMPLE_RATE) \
            .subscribe(on_next=lambda v: self.flock_repulsion.on_next(v))

    def flock_announce(self, name):
        if name != self.id and name not in self.flock_coordinates:
            self.log_publisher.publish("Flocking with {}".format(name))
            print("Registering flock member: {}".format(name))
            flock_coordinate_subject = Subject()
            self.flock_coordinates[name] = flock_coordinate_subject

            timeoutSubscription = None

            def coordinate_subscription_timeout():
                del self.flock_coordinates[name]
                self.subscribe_flock()
                timeoutSubscription.dispose()

            timeoutSubscription = flock_coordinate_subject.debounce(10000) \
                .subscribe(on_next=lambda v: coordinate_subscription_timeout())

            def flock_coordiante_subject(position):
                # print("name: {} position: {} {}".format(name, position.latitude, position.longitude))
                flock_coordinate_subject.on_next(position)

            self.ros_subscriptions.append(
                self.node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(name),
                                              flock_coordiante_subject, 10))

            self.flockSubscription.dispose()

            self.subscribe_flock()

    def step(self):
        if not self.started:
            self.started = True

            print("Subscribing...")

            self.ros_subscriptions.append(
                self.node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(self.leader),
                                              lambda position: self.leaderposition_subject.on_next(position), 10))
            self.ros_subscriptions.append(
                self.node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(self.id),
                                              lambda position: self.selfposition_subject.on_next(position), 10))
            self.ros_subscriptions.append(
                self.node.create_subscription(TwistStamped,
                                              "{}/mavros/local_position/velocity_local".format(self.leader),
                                              lambda twist: self.leadervelocity_subject.on_next(twist), 10))
            self.ros_subscriptions.append(
                self.node.create_subscription(TwistStamped, "{}/mavros/local_position/velocity_local".format(self.id),
                                              lambda twist: self.selfvelocity_subject.on_next(twist), 10))
            self.ros_subscriptions.append(
                self.node.create_subscription(String, "/dragonfly/announce", self.flock_announce_callback, 10))

            self.log_publisher.publish("Flocking")

        return ActionState.WORKING

    def stop(self):
        for subscription in self.ros_subscriptions:
            subscription.destroy()

        del self.ros_subscriptions

        self.navigate_subscription.dispose()
