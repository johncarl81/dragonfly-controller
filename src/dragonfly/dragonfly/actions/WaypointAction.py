#!/usr/bin/env python3
import math, rx, time
import rx.operators as ops
from rx.subject import Subject
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from dragonfly_messages.srv import Pump
from .ActionState import ActionState


def distance(position1, position2):
    deltax = position1.x - position2.x
    deltay = position1.y - position2.y
    deltaz = position1.z - position2.z

    return math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))


class WaypointAction:
    STOP_VELOCITY_THRESHOLD = 0.1
    SAMPLE_RATE = 1.0
    WAIT_FOR_WAYPOINT = 10
    WAYPOINT_ACCEPTANCE_ADJUSTMENT = {'x': 0, 'y': 0, 'z': 0}

    def __init__(self, logger, id, logPublisher, local_setposition_publisher, waypoint, distance_threshold,
                 local_velocity_observable, local_setvelocity_publisher, local_pose_observable, drone_stream_factory,
                 pump_service, run_pump=False, pump_threshold=math.inf, pump_num=-1, pump_duration=60):
        self.logger = logger
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
        self.waypointAcceptanceSubscription = rx.empty().subscribe()
        self.co2_monitor_subscription = rx.empty().subscribe()
        self.local_velocity_observable = local_velocity_observable
        self.local_pose_observable = local_pose_observable
        self.drone_stream_factory = drone_stream_factory
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.run_pump = run_pump
        self.pump_service = pump_service
        self.pump_num = pump_num
        self.pump_threshold = pump_threshold
        self.pump_duration = pump_duration
        self.pump_start = 0
        self.pump_setup = False
        self.pumping = False

    def step(self):
        if not self.pump_setup and self.run_pump:
            self_drone = self.drone_stream_factory.get_drone(self.id)
            self.co2_monitor_subscription = self_drone.get_co2().subscribe(
                on_next=self.monitor_co2,
                on_error=lambda e: self.logger.error(f"Error monitoring co2 threshold", e)
            )

            self.pump_setup = True
        if self.pumping:
            if self.pump_start == 0:
                self.pump_start = time.time()
                self.pump_service.call(Pump.Request(pump_num=self.pump_num))
            elif time.time() - self.pump_start > self.pump_duration:
                self.logger.info(f"Restarting waypoint")
                self.pumping = False
                # Re-command
                self.commanded = False

        if not self.commanded:
            self.commanded = True

            def updatePosition(pose, velocity, time):
                self.logger.debug(f"Distance to point:{self.waypoint.pose.position.x} "
                         f"{self.waypoint.pose.position.y} {self.waypoint.pose.position.z} - "
                         f"{distance(self.waypoint.pose.position, pose)}")
                alteredposition = pose

                alteredposition.x += WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['x']
                alteredposition.y += WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['y']
                alteredposition.z += WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['z']

                magnitude = math.sqrt((velocity.x * velocity.x) + (velocity.y * velocity.y) + (velocity.z * velocity.z))
                self.logger.debug(f"{magnitude} - {distance(self.waypoint.pose.position, alteredposition)} @ {time}")

                if distance(self.waypoint.pose.position, alteredposition) < self.distance_threshold:
                    self.status = ActionState.SUCCESS
                elif time > self.WAIT_FOR_WAYPOINT and magnitude < self.STOP_VELOCITY_THRESHOLD:
                    WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['x'] += self.waypoint.pose.position.x - pose.x
                    WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['y'] += self.waypoint.pose.position.y - pose.y
                    WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT['z'] += self.waypoint.pose.position.z - pose.z
                    self.logPublisher.publish(
                        String(data=f"Adjusted waypoint acceptance: {WaypointAction.WAYPOINT_ACCEPTANCE_ADJUSTMENT}"))

            self.waypointAcceptanceSubscription = rx.combine_latest(
                self.local_pose_observable, self.local_velocity_observable, rx.interval(1)).pipe(
                ops.sample(self.SAMPLE_RATE)
            ).subscribe(on_next=lambda values: updatePosition(values[0].pose.position, values[1].twist.linear, values[2]))

            self.local_setposition_publisher.publish(self.waypoint)

        return self.status

    def monitor_co2(self, message):
        if message.ppm > self.pump_threshold:
            self.logger.info(f"Reached co2 threshold: {message.ppm} > {self.pump_threshold}")
            self.stop()
            # Stop drone in position
            self.local_setvelocity_publisher.publish(TwistStamped())
            self.pumping = True


    def stop(self):
        self.waypointAcceptanceSubscription.dispose()
        self.co2_monitor_subscription.dispose()
