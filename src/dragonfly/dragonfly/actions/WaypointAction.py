#!/usr/bin/env python
import math

import rx
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import QoSProfile
from rx.core import Observable
from rx.subject import Subject

from .ActionState import ActionState


def distance(position1, position2):
    deltax = position1.x - position2.x
    deltay = position1.y - position2.y
    deltaz = position1.z - position2.z

    return math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))


class WaypointAction:
    STOP_VELOCITY_THRESHOLD = 0.1
    SAMPLE_RATE = 1000.0
    WAIT_FOR_WAYPOINT = 10
    WAYPOINT_ACCEPTANCE_ADJUSTMENT = {'x': 0, 'y': 0, 'z': 0}

    def __init__(self, id, logPublisher, local_setposition_publisher, waypoint, distance_threshold, node):
        self.id = id
        self.logPublisher = logPublisher
        self.waypoint = waypoint
        self.distance_threshold = distance_threshold
        self.local_setposition_publisher = local_setposition_publisher
        self.status = ActionState.WORKING
        self.commanded = False
        self.position_update = None
        self.velocity_update = None
        self.velocity_subject = Subject()
        self.pose_subject = Subject()
        self.waypointAcceptanceSubscription = Observable.empty().subscribe()
        self.node = node

    def step(self):
        if not self.commanded:
            self.commanded = True

            def updatePosition(poseVelocity):

                # print("Distance to point:{} {} {}".format(self.waypoint.pose.position.x, self.waypoint.pose.position.y, self.waypoint.pose.position.z), \
                #       distance(self.waypoint.pose.position, localposition.pose.position))
                time = poseVelocity['time']
                alteredposition = poseVelocity['pose']

                alteredposition.x += WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['x']
                alteredposition.y += WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['y']
                alteredposition.z += WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['z']

                velocity = poseVelocity['velocity']

                magnitude = math.sqrt((velocity.x * velocity.x) + (velocity.y * velocity.y) + (velocity.z * velocity.z))
                # print("{} - {} @ {}".format(magnitude, distance(self.waypoint.pose.position, alteredposition), time))

                if distance(self.waypoint.pose.position, alteredposition) < self.distance_threshold:
                    self.status = ActionState.SUCCESS

                    self.stop()
                elif time > self.WAIT_FOR_WAYPOINT and magnitude < self.STOP_VELOCITY_THRESHOLD:
                    WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['x'] += self.waypoint.pose.position.x - poseVelocity[
                        'pose'].x
                    WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['y'] += self.waypoint.pose.position.y - poseVelocity[
                        'pose'].y
                    WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['z'] += self.waypoint.pose.position.z - poseVelocity[
                        'pose'].z
                    self.logPublisher.publish(
                        "Adjusted waypoint acceptance: {}".format(WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT))

            self.position_update = self.node.create_subscription(PoseStamped,
                                                                 "{}/mavros/local_position/pose".format(self.id),
                                                                 lambda pose: self.pose_subject.on_next(pose),
                                                                 qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))
            self.velocity_update = self.node.create_subscription(TwistStamped,
                                                                 "{}/mavros/local_position/velocity_local".format(
                                                                     self.id),
                                                                 lambda velocity: self.velocity_subject.on_next(
                                                                     velocity),
                                                                 qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

            self.waypointAcceptanceSubscription = Observable.combine_latest(self.pose_subject, self.velocity_subject,
                                                                            rx.Observable.interval(1000),
                                                                            lambda position, velocity,
                                                                                   time: self.combinePoseVelocity(
                                                                                position, velocity, time)) \
                .sample(self.SAMPLE_RATE) \
                .subscribe(on_next=lambda pose_velocity: updatePosition(pose_velocity))

            self.local_setposition_publisher.publish(self.waypoint)

        return self.status

    def combinePoseVelocity(self, position, velocity, time):
        return {'pose': position.pose.position, 'velocity': velocity.twist.linear, 'time': time}

    def stop(self):
        if self.position_update is not None:
            self.position_update.destroy()
            self.position_update = None
        if self.velocity_update is not None:
            self.velocity_update.destroy()
            self.velocity_update = None
        self.waypointAcceptanceSubscription.dispose()
