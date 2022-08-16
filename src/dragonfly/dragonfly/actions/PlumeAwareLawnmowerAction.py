#!/usr/bin/env python3
import math
import rx
import rx.operators as ops
from std_msgs.msg import String
from .ActionState import ActionState


def distance(position1, position2):
    deltax = position1.x - position2.x
    deltay = position1.y - position2.y
    deltaz = position1.z - position2.z

    return math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))


class PlumeAwareLawnmowerAction:
    STOP_VELOCITY_THRESHOLD = 0.1
    SAMPLE_RATE = 0.1
    WAIT_FOR_WAYPOINT = 10
    WAYPOINT_ACCEPTANCE_ADJUSTMENT = {'x': 0, 'y': 0, 'z': 0}

    def __init__(self, id, logPublisher, local_setposition_publisher, waypoints, distance_threshold, step_length, boundary_length, local_pose_observable, co2_observable):
        self.id = id
        self.logPublisher = logPublisher
        self.waypoints = waypoints
        self.distance_threshold = distance_threshold
        self.local_setposition_publisher = local_setposition_publisher
        self.waypointAcceptanceSubscription = rx.empty().subscribe()
        self.local_pose_observable = local_pose_observable
        self.co2_observable = co2_observable
        self.step_length = step_length
        self.boundary_length = boundary_length

        self.current_waypoint_index = 0
        self.current_waypoint = self.waypoints[0]
        self.above_ambient_last = None
        self.ambient_threshold = 425
        self.status = ActionState.WORKING
        self.commanded = False

    def is_pass(self):
        return self.current_waypoint_index > self.boundary_length and \
               (self.current_waypoint_index - self.boundary_length) % 2 == 1

    def is_between_next_pass(self, x):
        pass_start_x = self.waypoints[self.current_waypoint_index + 1].pose.position.x
        pass_end_x = self.waypoints[self.current_waypoint_index + 2].pose.position.x
        if pass_start_x > pass_end_x:
            pass_start_x, pass_end_x = pass_end_x, pass_start_x

        return pass_start_x < x < pass_end_x

    def step(self):
        if self.current_waypoint_index < len(self.waypoints) :
            if not self.commanded:
                self.commanded = True

                def updatePosition(pose, ppm):
                    if self.is_pass():
                        if ppm > self.ambient_threshold:
                            self.above_ambient_last = pose

                        if self.above_ambient_last is not None and \
                                distance(pose, self.above_ambient_last) > self.step_length and \
                                self.current_waypoint_index < len(self.waypoints) - 2:
                            # Turn, below ambient
                            if self.is_between_next_pass(pose.x):
                                self.current_waypoint_index = self.current_waypoint_index + 1
                                self.current_waypoint = self.waypoints[self.current_waypoint_index]
                                self.current_waypoint.pose.position.x = pose.x
                                self.logPublisher.publish(String(data="Exited plume, pruning..."))
                                self.local_setposition_publisher.publish(self.current_waypoint)
                                self.above_ambient_last = None

                    if distance(self.current_waypoint.pose.position, pose) < self.distance_threshold:
                        if self.current_waypoint_index < len(self.waypoints) - 1:
                            self.current_waypoint_index = self.current_waypoint_index + 1
                            self.current_waypoint = self.waypoints[self.current_waypoint_index]
                            self.local_setposition_publisher.publish(self.current_waypoint)
                            self.logPublisher.publish(String(data="Goto {} {}/{}".format(
                                "Lawnmower",
                                self.current_waypoint_index + 1,
                                len(self.waypoints))))
                        else:
                            self.status = ActionState.SUCCESS
                            self.stop()

                self.waypointAcceptanceSubscription = rx.combine_latest(self.local_pose_observable, self.co2_observable).pipe(
                    ops.sample(self.SAMPLE_RATE)
                ).subscribe(on_next=lambda values: updatePosition(values[0].pose.position, values[1].ppm))

                self.local_setposition_publisher.publish(self.current_waypoint)
        return self.status

    def stop(self):
        self.waypointAcceptanceSubscription.dispose()
