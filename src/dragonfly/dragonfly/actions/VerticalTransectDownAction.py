#!/usr/bin/env python3
import rx
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from .ActionState import ActionState

class VerticalTransectDownAction:
    DOWNWARD_VELOCITY = 1.0

    def __init__(self, logger, id, logPublisher, drone_stream_factory, minimum_altitude, local_setvelocity_publisher):

        self.logger = logger
        self.logPublisher = logPublisher
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.range_finder_subject = drone_stream_factory.get_drone(id).get_rangefinder()

        self.range_finder_subscription = rx.empty().subscribe()
        self.minimum_altitude = minimum_altitude
        self.commanded = False
        self.reached_minimum = False
        self.status = ActionState.WORKING

    def step(self):
        if not self.commanded:
            self.commanded = True
            self.range_finder_subscription = self.range_finder_subject.subscribe(on_next = lambda range: self.check_minimum_range(range))

        if self.status == ActionState.WORKING and self.reached_minimum:
            self.local_setvelocity_publisher.on_next(TwistStamped()) # Stop
            self.status = ActionState.SUCCESS
        else:
            self.navigate_down()

        return self.status

    def check_minimum_range(self, range):
        if 0 < range.range <= self.minimum_altitude:
            self.logPublisher.publish(String(data="Reached minimum altitude."))
            self.reached_minimum = True

    def navigate_down(self):
        twist = TwistStamped()
        twist.twist.linear.z = -self.DOWNWARD_VELOCITY
        self.local_setvelocity_publisher.publish(twist)

    def stop(self):
        self.range_finder_subscription.dispose()
