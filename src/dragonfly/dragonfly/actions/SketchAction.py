#!/usr/bin/env python3
import math
import random
import rx
import rx.operators as ops
import numpy as np

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from rx.subject import Subject
from dragonfly_messages.msg import LatLon, PositionVector

from .ActionState import ActionState

EARTH_CIRCUMFRENCE = 40008000

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
        self.target_position_vector = None
        self.encountered = False

    def navigate(self, input):

        twist = TwistStamped()

        for vector in input:
            twist.twist.linear.x += vector[0]
            twist.twist.linear.y += vector[1]

        self.local_setvelocity_publisher.publish(twist)

    def differenceInMeters(self, one, two):
        return [
            ((one.longitude - two.longitude) * (EARTH_CIRCUMFRENCE / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (EARTH_CIRCUMFRENCE / 360))
        ]

    def format_velocities(self, twist):
        return [
            twist.twist.linear.x,
            twist.twist.linear.y
        ]

    def magnitude(self, vector):
        return np.linalg.norm(vector)

    def step(self):
        if not self.started:
            self.started = True

            print("Subscribing...")

            partner_drone = self.drone_stream_factory.get_drone(self.partner)
            self_drone = self.drone_stream_factory.get_drone(self.id)

            self.navigate_subscription = rx.combine_latest(
                partner_drone.get_position(),
                self_drone.get_position(),
                self.dragonfly_sketch_subject
            ).pipe(
                ops.sample(self.SAMPLE_RATE),
                ops.map(lambda positions: self.tandem(positions[0], positions[1], positions[2]))
            ).subscribe(lambda vectors: self.navigate(vectors))

            if self.leader:
                self.leader_broadcast_subscrition = rx.combine_latest(
                    self.setupSubject(self.partner),
                    self.setupSubject(self.id)
                ).pipe(
                    ops.sample(self.SAMPLE_RATE)
                ).subscribe(lambda positionReadingVector: self.broadcast_target(positionReadingVector[0], positionReadingVector[1]))

            self.log_publisher.publish(String(data="Sketch"))

        return ActionState.WORKING

    def setupSubject(self, drone):

        drone_streams = self.drone_stream_factory.get_drone(drone)

        position_subject = drone_streams.get_position()
        co2_subject = drone_streams.get_co2()

        position_value_subject = Subject()

        rx.combine_latest(position_subject, co2_subject).pipe(
            ops.map(lambda tuple, offset=drone_streams.mean: ReadingPosition(tuple[0].latitude, tuple[0].longitude, tuple[1].ppm - offset))
        ).subscribe(on_next=lambda v: position_value_subject.on_next(v))

        return position_value_subject

    def tandem(self, partner_position, self_position, positionVector):

        tandem_offset = self.caculate_tandem_offset(partner_position, self_position)
        tandem_angle = self.calculate_tandem_angle(partner_position, self_position, positionVector)
        straight_line = self.calculate_straight_line(positionVector)
        turning = self.calculate_turn(partner_position, self_position, positionVector)
        offset_error_correction = self.calcuate_error_correction(partner_position, self_position, positionVector)

        # print(f"{self.id} tandem_offset: {tandem_offset} tandem_angle: {tandem_angle} straight_line: {straight_line}")

        return [tandem_offset, tandem_angle, straight_line, turning, offset_error_correction]

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

    def calculate_tandem_angle(self, partner_position, self_position, positionVector):
        if positionVector.movement == PositionVector.FORWARD:
            distance_m = self.differenceInMeters(self_position, partner_position)
            offset_unitary = self.unitary(distance_m)

            direction_vector = [positionVector.x, positionVector.y]

            return -2 * (np.dot(offset_unitary, direction_vector)) * np.array(direction_vector)
        else:
            return [0, 0]

    def calculate_straight_line(self, positionVector):
        if positionVector.movement == PositionVector.FORWARD:
            return [3 * positionVector.x, 3 * positionVector.y]
        else:
            return [0, 0]

    def rotate_vector(self, vector, angle):
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        return np.dot(rotation_matrix, vector)


    def calculate_turn(self, partner_position, self_position, positionVector):
        if positionVector.movement == PositionVector.TURN:

            self_center = self.differenceInMeters(positionVector.center, self_position)
            self_center_distance = self.magnitude(self_center)
            partner_center_distance = self.magnitude(self.differenceInMeters(positionVector.center, partner_position))

            if self_center_distance > partner_center_distance:
                velocity = 3
            else:
                velocity = (self_center_distance / partner_center_distance) * 3

            vector_to_center = self.unitary(self_center)

            starting_position = self.differenceInMeters(positionVector.center, positionVector.position)
            # print(f"{self_center} {starting_position}")
            angle = math.atan2(self_center[1], self_center[0]) - math.atan2(starting_position[1], starting_position[0])

            rotated_vector = self.rotate_vector([positionVector.x, positionVector.y], angle)

            # print(f"Angle: {angle}, rotated_vector: {rotated_vector}")

            forward_force = np.array([rotated_vector[0] * velocity, rotated_vector[1] * velocity])
            centripedal_force = np.array([vector_to_center[0] * (velocity / self_center_distance), vector_to_center[1] * (velocity / self_center_distance)])

            # print(f"forward_force: {forward_force}, centripedal_force: {centripedal_force}")
            # print(f"hyp: {hyp} target_offset: {target_offset}")

            return forward_force + centripedal_force
        else:
            return [0, 0]

    def calcuate_error_correction(self, partner_position, self_position, positionVector):
        if positionVector.movement == PositionVector.FORWARD:
            lat_ave = (self_position.latitude + partner_position.latitude) / 2
            lon_ave = (self_position.longitude + partner_position.longitude) / 2

            vector_to_target =  self.differenceInMeters(dotdict({'latitude': lat_ave, 'longitude': lon_ave}), positionVector.position)

            dot_product = np.dot([positionVector.x, positionVector.y], vector_to_target)

            vector_to_line = [
                vector_to_target[0] - (dot_product * positionVector.x),
                vector_to_target[1] - (dot_product * positionVector.y)
            ]

            magnitude = self.magnitude(vector_to_line)

            if magnitude == 0:
                return [0, 0]
            else:
                return vector_to_line / self.magnitude(vector_to_line)
        else:
            return [0, 0]

    def unitary(self, vector):
        magnitude = self.magnitude(vector)
        return [vector[0] / magnitude, vector[1] / magnitude]

    def average(self, one, two):
        return [(one.longitude + two.longitude)/2, (one.latitude + two.latitude) / 2]

    def broadcast_target(self, partner_position, self_position):

        average_position =  dotdict({'latitude':  (self_position.latitude + partner_position.latitude) / 2,
                                     'longitude': (self_position.longitude + partner_position.longitude) / 2})

        if not self.target_position_vector:
            positionVector = PositionVector()
            positionVector.movement = PositionVector.FORWARD
            position = LatLon()
            position.longitude = average_position.longitude
            position.latitude = average_position.latitude
            positionVector.position = position
            positionVector.x = 0.0
            positionVector.y = 1.0
            positionVector.position = position
            positionVector.distance = 10.0

            self.target_position_vector = positionVector

        if not self.encountered and (self.inside(partner_position) or self.inside(self_position)):
            self.encountered = True
            # print(f"Encountered plume: {partner_position.value} {self_position.value}")

        if self.encountered and self.passed(average_position):
            # print(f"PASSED! {positionVector} | {self.target_position_vector}")
            self.target_position_vector = self.boundary_sketch(partner_position, self_position)
        self.position_vector_publisher.publish(self.target_position_vector)
        self.dragonfly_sketch_subject.on_next(self.target_position_vector)

# Algorithm 1 Ensures the robots are at distance √
#
# λ from each other and are oriented in the same direction.
# Additional discussion with illustrative diagrams of this synchronization is in Appendix D.
#
# procedure SYNCHRONIZE(D1, D2)
#   Path ← the polyline path of D2 from last crossing of BOUNDARY-SKETCH with the shape till current position.
#   ∇ ← the gradient at the last boundary crossing for D2.
#   L1 ← the line in the direction of ∇ through D1’s position.
#   L2 ← the line in the direction of ∇ through D2’s position.
#   if L1 crosses Path then
#       Move D2 in its current direction until it is √λ distance away from L1.
#       Change direction to ∇ and take a single step of length λ.
#       Move D1 along L1 until it is √λ away from D2.
#
#   else
#       Move D1 in its current direction until it is √ λ distance away from L2.
#
#   Change direction to ∇ and move until the distance from D2 is √λ.

# Algorithm 2 Reestablishes “Sandwich” Invariant
# procedure CROSS-BOUNDARY(D1, D2, α)
#   p ← last position of D1 before crossing
#   R ← the vertices of the regular polygon including D1’s position with exterior angle √λ and
#       the edge beginning at D1’s position facing the direction of ∇ + α.
#   P ← the vertices of the convex hull of R ∪ {p}. For all i : 0 ≤ i ≤ |P| − 1, let
#       Pi be the i-th vertex in this convex hull, ordered such that P0 = p and P1 = D1’s current position.
#   ∇ ← gradient at the last boundary crossing of D1
#   i ← 1.
#   while neither robot has crossed the boundary AND i + 1 < |P| do
#       D1 moves to Pi+1.
#       D2 moves to closest point from it that is √λ distance away from Pi and orthogonal to ∇ + iα
#       i ← i + 1
#   while neither robot has crossed the boundary do
#       D1 moves towards point p taking steps of length λ.
#       D2 moves to closest point from it that is √λ distance away from D1 and orthogonal to D1’s direction.
#   if D2 crossed the boundary then
#       SYNCHRONIZE (D1, D2)
#   else
#       ∇ ← the current direction of D1.



    def cross_boundary(self, d1, d2, a):
        return self.turn(d1, d2, a)

    # Algorithm 3 Initially, robots are √λ apart; one inside and one outside
    def boundary_sketch(self, d1, d2):
        # D1, D2 ← the two robots
        # ∇ ← boundary gradient at point of crossing with line segment between D1 and D2
        # α ← √λ
        lambda_value = 0.01
        if self.inside(d1) ^ self.inside(d2):
            # D1 and D2 both move λ distance in the direction of ∇
            print("sandwich")
            return self.forward(d1, d2)

        if not self.inside(d1) and not self.inside(d2):
            print("outside")
            a = -math.sqrt(lambda_value)
            return self.cross_boundary(d1, d2, a)
        elif self.inside(d1) and self.inside(d2):
            print("inside")
            a = math.sqrt(lambda_value)
            return self.cross_boundary(d1, d2, a)

    def calculate_direction(self):
        if self.target_position_vector.movement == PositionVector.FORWARD:
            direction = [self.target_position_vector.x, self.target_position_vector.y]
            print(f"forward: {direction}")
            return direction
        if self.target_position_vector.movement == PositionVector.TURN:
            prev_vector = [self.target_position_vector.x, self.target_position_vector.y]
            angle =  self.target_position_vector.a * self.target_position_vector.p + (math.pi / 2)
            direction = self.rotate_vector(prev_vector, angle)
            print(f"turn {angle}: {direction}")
            return direction

    def forward(self, d1, d2):
        average_position = self.average(d1, d2)

        positionVector = PositionVector()
        positionVector.movement = PositionVector.FORWARD
        position = LatLon()
        position.longitude = average_position[0]
        position.latitude = average_position[1]
        positionVector.position = position
        [positionVector.x, positionVector.y] = self.calculate_direction()
        positionVector.position = position
        positionVector.distance = 10.0

        return positionVector

    def turn(self, d1, d2, a):
        if a == self.target_position_vector.a:
            self.target_position_vector.p += 1
            return self.target_position_vector
        average_position = self.average(d1, d2)
        distance_m = self.differenceInMeters(d1, d2)
        offset_unitary = self.unitary(distance_m)

        positionVector = PositionVector()
        positionVector.movement = PositionVector.TURN
        position = LatLon()
        position.longitude = average_position[0]
        position.latitude = average_position[1]
        positionVector.position = position
        [positionVector.x, positionVector.y] = self.calculate_direction()
        positionVector.position = position
        positionVector.radius = 10.0
        positionVector.a = a
        positionVector.p = 1

        hyp = 1 / np.arcsin(a)
        print(hyp)

        center = LatLon()
        center.longitude = average_position[0] + ((offset_unitary[0] * hyp) / (math.cos(average_position[1] * 0.01745) * (EARTH_CIRCUMFRENCE / 360)))
        center.latitude = average_position[1] - ((offset_unitary[1] * hyp) / (EARTH_CIRCUMFRENCE / 360))

        print(f"center lon: {center.longitude} lat: {center.latitude}")

        positionVector.center = center

        return positionVector

    def inside(self, d):
        return d.value > 421

    def passed(self, average_position):
        if self.target_position_vector is None:
            return True

        if self.target_position_vector.movement == PositionVector.FORWARD:

            target_offset = self.differenceInMeters(average_position, self.target_position_vector.position)

            return np.dot(target_offset, [self.target_position_vector.x, self.target_position_vector.y]) > self.target_position_vector.distance
        else:
            target_offset = self.magnitude(self.differenceInMeters(average_position, self.target_position_vector.position))
            hyp = self.magnitude(self.differenceInMeters(average_position, self.target_position_vector.center))

            # print(f"({target_offset} / {hyp}) > math.sin({self.target_position_vector.a} * {self.target_position_vector.p}) = {(target_offset / hyp) > math.sin(self.target_position_vector.a * self.target_position_vector.p)}")
            return (target_offset / hyp) > math.sin(self.target_position_vector.a * self.target_position_vector.p)
            # return False

    def stop(self):
        self.navigate_subscription.dispose()
        self.leader_broadcast_subscrition.dispose()
