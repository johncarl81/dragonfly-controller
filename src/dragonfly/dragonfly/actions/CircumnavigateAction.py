#! /usr/bin/env python3
import math, rx, time
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from .ActionState import ActionState
import rx.operators as ops

class CircumnavigateAction:

    SAMPLE_RATE = 0.1
    k0 = 0.45
    k1 = 0.5
    k2 = 1
    k3 = 2.5
    a = np.matrix([0, 0, 1]).transpose()
    a_a_transpose = a * a.transpose()
    rho = 4.2

    def __init__(self, id, logger, local_setvelocity_publisher, target, flock, drone_stream_factory):
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.id = id
        self.logger = logger
        self.target = target
        self.started = False
        self.flock = flock
        self.drone_stream_factory = drone_stream_factory

        self.twist = TwistStamped()
        self.previous_time = None

        self.n = len(flock)

        self.id_index = -1
        for i in range(self.n):
            if flock[i] == id:
                self.id_index = i

        # Ring topology adjacency matrix
        self.A = np.zeros(self.n)
        for j in range(self.n):
            offset = abs(self.id_index - j) % self.n
            if offset == 1 or offset == self.n - 1:
                self.A[j] = 1

        # Desired phasing angle between agents (thetaij may be different from thetaik)
        self.thetaij = 2 * math.pi / self.n

        # Desired inter-agent distance
        self.dij = 2 * self.rho * math.sin(self.thetaij / 2)

        # Matrix of inter-agent distances
        self.D = self.dij * self.A

        self.ros_subscriptions = []

        self.circumnavigate_subscription = rx.empty().subscribe()


    def differenceInMeters(self, one, two):
        earthCircumference = 40008000
        return [
            ((one.longitude - two.longitude) * (earthCircumference / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (earthCircumference / 360)),
            one.altitude
        ]

    def format_velocities(self, twist):
        self.logger.info("format_velocities")
        return [
            twist.twist.linear.x,
            twist.twist.linear.y,
            twist.twist.linear.z
        ]

    def relative_to_target(self, elements):
        self.logger.info("relative_to_target")
        targetLatLon = elements[1]

        converted_elements = [
            [self.differenceInMeters(element, targetLatLon) for element in elements[0]],
            [0, 0, targetLatLon.altitude],
            self.format_velocities(elements[2])]

        return converted_elements

    def velocity_vector(self, elements):
        self.logger.info("velocity_vector")

        pt = np.matrix(elements[1]).transpose()

        p_i = np.matrix(elements[0][self.id_index]).transpose()

        pit = p_i - pt

        DeliV1 = self.a_a_transpose * pit

        # Orthogonal projection matrix of a
        Pa = np.eye(3, 3) - self.a_a_transpose
        phii = (pit / np.linalg.norm(pit))
        phiia = (Pa * phii) / np.linalg.norm(Pa * phii)
        DeliV2 = (np.linalg.norm(Pa * pit) - self.rho) * phiia

        DeliV3 = np.matrix([0.0, 0.0, 0.0])


        for j in range(self.n):
            pj = np.matrix(elements[0][j]).transpose()
            pjt = pj - pt
            phij = pjt / np.linalg.norm(pjt)
            phija = Pa * phij / np.linalg.norm(Pa*phij)

            gammaij = self.A[j] * ((np.linalg.norm(phiia - phija) ** 2) - ((self.D[j] ** 2) / (self.rho ** 2))) * np.cross(self.a.transpose(), phiia.transpose()) * (phiia - phija)

            DeliV3 += (1 / np.linalg.norm(Pa * pit)) * gammaij * np.cross(self.a.transpose(), phiia.transpose())

        ui1 = -self.k1 * DeliV1
        ui2 = (-self.k2 * DeliV2) + (self.k0 * np.linalg.norm(Pa * pit) * np.cross(self.a.transpose(), phiia.transpose()).transpose())
        ui3 = -self.k3 * np.linalg.norm(Pa * pit) * DeliV3.transpose()

        ptdot = np.matrix(elements[2]).transpose()

        ui = ui1 + ui2 + ui3 + ptdot

        self.logger.info(f"ui: {ui1} + {ui2} + {ui3} + {ptdot}")

        return np.asarray(ui).flatten()

    def navigate(self, input):
        self.logger.info("navigate")

        twist = TwistStamped()
        twist.twist.linear.x = input[0]
        twist.twist.linear.y = input[1]
        twist.twist.linear.z = input[2]

        self.local_setvelocity_publisher.publish(twist)


    def step(self):
        if not self.started:
            self.started = True

            self.logger.info("Subscribing...")

            targetposition_subject = self.drone_stream_factory.get_drone(self.target).get_position()
            self.logger.info(f"Subscribed to {self.target}/mavros/global_position/global")
            targetvelocity_subject = self.drone_stream_factory.get_drone(self.target).get_velocity()
            self.logger.info(f"Subscribed to {self.target}/mavros/local_position/velocity_local")

            flockposition_subjects = []

            for name in self.flock:
                flockposition_subjects.append(self.drone_stream_factory.get_drone(name).get_position())
                self.logger.info(f"Subscribed to {name}/mavros/global_position/global")

            self.circumnavigate_subscription = rx.combine_latest(
                rx.combine_latest(*flockposition_subjects),
                targetposition_subject,
                targetvelocity_subject).pipe(
                    ops.sample(self.SAMPLE_RATE),
                    ops.map(self.relative_to_target),
                    ops.map(self.velocity_vector)
                ).subscribe(on_next = self.navigate)

            self.logger.info("Setup finished")

        return ActionState.WORKING

    def stop(self):
        del self.ros_subscriptions
        self.circumnavigate_subscription.dispose()
