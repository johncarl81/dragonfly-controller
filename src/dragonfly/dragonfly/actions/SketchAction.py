#!/usr/bin/env python3
import math
import rx
import rx.operators as ops

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from dragonfly_messages.msg import LatLon, PositionVector

from .ActionState import ActionState


class SketchAction:
    SAMPLE_RATE = 0.1

    def __init__(self, id, log_publisher, local_setvelocity_publisher, announce_stream, offset, partner, leader,
                 drone_stream_factory, dragonfly_sketch_subject, position_vector_publisher):
        self.log_publisher = log_publisher
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.id = id
        self.partner = partner
        self.offset = offset
        self.leader = leader
        self.started = False
        self.ros_subscriptions = []
        self.announce_stream = announce_stream
        self.drone_stream_factory = drone_stream_factory
        self.dragonfly_sketch_subject = dragonfly_sketch_subject
        self.position_vector_publisher = position_vector_publisher

        self.navigate_subscription = rx.empty().subscribe()
        self.leader_broadcast_subscrition = rx.empty().subscribe()

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

    def format_velocities(self, twist):
        return [
            twist.twist.linear.x,
            twist.twist.linear.y
        ]

    def magnitude(self, vector):
        return math.sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]))

    def step(self):
        if not self.started:
            self.started = True

            print("Subscribing...")

            partner_drone = self.drone_stream_factory.get_drone(self.partner)
            self_drone = self.drone_stream_factory.get_drone(self.id)
            partner_position_subject = partner_drone.get_position()
            self_position_subject = self_drone.get_position()
            partner_velocity_subject = partner_drone.get_velocity()
            self_velocity_subject = self_drone.get_velocity()

            self.navigate_subscription = rx.combine_latest(partner_position_subject, self_position_subject, self.dragonfly_sketch_subject).pipe(
                ops.sample(self.SAMPLE_RATE),
                ops.map(lambda positions: self.tandem(positions[0], positions[1], positions[2]))
            ).subscribe(lambda vectors: self.navigate(vectors))

            if self.leader:
                self.leader_broadcast_subscrition = rx.combine_latest(rx.zip(partner_position_subject, self_position_subject).pipe(
                    ops.take(1),
                    ops.map(lambda positions: self.generate_position_vector(positions[0], positions[1]))
                ), rx.interval(1)).pipe(
                    ops.map(lambda values: values[0])
                ).subscribe(lambda positionVector: self.broadcast_target(positionVector))

            self.log_publisher.publish(String(data="Sketch"))

        return ActionState.WORKING

    def tandem(self, partner_position, self_position, positionVector):

        tandem_offset = self.caculate_tandem_offset(partner_position, self_position)
        tandem_angle = self.caculate_tandem_angle(partner_position, self_position, positionVector)
        straignt_line = self.calculate_straight_line(positionVector)
        turning = [0, 0]
        offset_error_correction = [0, 0]

        print(f"{self.id} tandem_offset: {tandem_offset} straignt_line: {straignt_line}")

        return [tandem_offset, tandem_angle, straignt_line, turning, offset_error_correction]

    def caculate_tandem_offset(self, partner_position, self_position):
        distance_m = self.differenceInMeters(partner_position, self_position)
        offset_unitary = self.unitary(distance_m)

        possition_offset = [distance_m[0] - (offset_unitary[0] * self.offset),
                            distance_m[1] - (offset_unitary[1] * self.offset)]
        offset_magnitude = self.magnitude(possition_offset)

        tandem_distance = [
            2 * possition_offset[0] / max(2, offset_magnitude),
            2 * possition_offset[1] / max(2, offset_magnitude)
        ]
        return tandem_distance

    def caculate_tandem_angle(self, partner_position, self_position, positionVector):
        return [0, 0]

    def calculate_straight_line(self, positionVector):
        return [3 * positionVector.x, 3 * positionVector.y]

    def unitary(self, vector):
        magnitude = self.magnitude(vector)
        return [vector[0] / magnitude, vector[1] / magnitude]

    def average(self, one, two):
        return [(one.longitude + two.longitude)/2, (one.latitude + two.latitude) / 2]

    def generate_position_vector(self, partner_position, self_position):
        print(f"parner_position: {partner_position}, self_position: {self_position}")
        average_position = self.average(partner_position, self_position)
        distance_m = self.differenceInMeters(partner_position, self_position)
        offset_unitary = self.unitary(distance_m)

        positionVector = PositionVector()
        position = LatLon()
        position.longitude = average_position[0]
        position.latitude = average_position[1]
        positionVector.position = position
        positionVector.x = offset_unitary[1]
        positionVector.y = -offset_unitary[0]
        return positionVector

    def broadcast_target(self, positionVector):
        print(f"outgoing positionVector: {positionVector}")

        self.position_vector_publisher.publish(positionVector)
        self.dragonfly_sketch_subject.on_next(positionVector)

    def stop(self):
        self.navigate_subscription.dispose()
        self.leader_broadcast_subscrition.dispose()
