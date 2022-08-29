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

    def __init__(self, name, id, logPublisher, waypoints, boundary_length, parameters, local_setposition_publisher, local_pose_observable, local_velocity_observable, co2_observable):
        self.name = name
        self.id = id
        self.logPublisher = logPublisher
        self.waypoints = waypoints
        self.parameters = parameters
        self.local_setposition_publisher = local_setposition_publisher
        self.waypointAcceptanceSubscription = rx.empty().subscribe()
        self.local_pose_observable = local_pose_observable
        self.local_velocity_observable = local_velocity_observable
        self.co2_observable = co2_observable
        self.boundary_length = boundary_length

        self.current_waypoint_index = 0
        self.current_waypoint = self.waypoints[0]
        self.above_ambient_last = None
        self.status = ActionState.WORKING
        self.commanded = False
        self.last_waypoint_time = 0

    def is_pass(self):
        return self.current_waypoint_index > self.boundary_length and \
               (self.current_waypoint_index - self.boundary_length) % 2 == 1

    def is_between_next_pass(self, x):
        pass_start_x = self.waypoints[self.current_waypoint_index + 1].pose.position.x
        pass_end_x = self.waypoints[self.current_waypoint_index + 2].pose.position.x
        if pass_start_x > pass_end_x:
            pass_start_x, pass_end_x = pass_end_x, pass_start_x

        return pass_start_x < x < pass_end_x

    def interpolate_y(self, x, one, two):
        y1 = one.pose.position.y
        y2 = two.pose.position.y
        x1 = one.pose.position.x
        x2 = two.pose.position.x

        return y1 + ((y2 - y1) * ((x - x1) / (x2 - x1)))

    def step(self):
        if self.current_waypoint_index < len(self.waypoints) :
            if not self.commanded:
                self.commanded = True

                def updatePosition(pose, velocity, ppm, time):
                    alteredposition = pose

                    alteredposition.x += PlumeAwareLawnmowerAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['x']
                    alteredposition.y += PlumeAwareLawnmowerAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['y']
                    alteredposition.z += PlumeAwareLawnmowerAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['z']

                    if self.parameters.co2_limit and self.is_pass():
                        if ppm > self.parameters.co2_threshold:
                            self.above_ambient_last = alteredposition

                        if self.above_ambient_last is not None and \
                                distance(alteredposition, self.above_ambient_last) > self.parameters.co2_limit_margin and \
                                self.current_waypoint_index < len(self.waypoints) - 2:
                            # Turn, below ambient
                            if self.is_between_next_pass(alteredposition.x):
                                self.current_waypoint_index = self.current_waypoint_index + 1
                                self.current_waypoint = self.waypoints[self.current_waypoint_index]
                                self.current_waypoint.pose.position.x = alteredposition.x
                                y = self.interpolate_y(alteredposition.x,
                                                       self.waypoints[self.current_waypoint_index],
                                                       self.waypoints[self.current_waypoint_index + 1])
                                self.current_waypoint.pose.position.y = y
                                self.logPublisher.publish(String(data="Exited plume, pruning..."))
                                self.local_setposition_publisher.publish(self.current_waypoint)
                                self.above_ambient_last = None


                    magnitude = math.sqrt((velocity.x * velocity.x) + (velocity.y * velocity.y) + (velocity.z * velocity.z))

                    if distance(self.current_waypoint.pose.position, alteredposition) < self.parameters.distance_threshold:
                        if self.current_waypoint_index < len(self.waypoints) - 1:
                            self.current_waypoint_index = self.current_waypoint_index + 1
                            self.current_waypoint = self.waypoints[self.current_waypoint_index]
                            self.local_setposition_publisher.publish(self.current_waypoint)
                            self.last_waypoint_time = time
                            self.logPublisher.publish(String(data="Goto {} {}/{}".format(
                                self.name,
                                self.current_waypoint_index + 1,
                                len(self.waypoints))))
                        else:
                            self.status = ActionState.SUCCESS
                            self.stop()
                    elif (time - self.last_waypoint_time) > self.WAIT_FOR_WAYPOINT and magnitude < self.STOP_VELOCITY_THRESHOLD:
                        PlumeAwareLawnmowerAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['x'] += self.current_waypoint.pose.position.x - pose.x
                        PlumeAwareLawnmowerAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['y'] += self.current_waypoint.pose.position.y - pose.y
                        PlumeAwareLawnmowerAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['z'] += self.current_waypoint.pose.position.z - pose.z
                        self.logPublisher.publish(
                        String(data="Adjusted waypoint acceptance: {}".format(PlumeAwareLawnmowerAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT)))

                self.waypointAcceptanceSubscription = rx.combine_latest(self.local_pose_observable,
                                                                        self.local_velocity_observable,
                                                                        self.co2_observable,
                                                                        rx.interval(1)).pipe(
                    ops.sample(self.SAMPLE_RATE)
                ).subscribe(on_next=lambda values: updatePosition(values[0].pose.position, values[1].twist.linear, values[2].ppm, values[3]))

                self.local_setposition_publisher.publish(self.current_waypoint)
        return self.status

    def stop(self):
        self.waypointAcceptanceSubscription.dispose()
