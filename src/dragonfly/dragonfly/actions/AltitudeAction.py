#!/usr/bin/env python
import math
import rx

from geometry_msgs.msg import PoseStamped

from .ActionState import ActionState


def distance(position1, position2):
    deltax = position1.x - position2.x
    deltay = position1.y - position2.y
    deltaz = position1.z - position2.z

    return math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))


class AltitudeAction:

    def __init__(self, id, local_setposition_publisher, altitude, distance_threshold, local_position_observable):
        self.id = id
        self.local_position_observable = local_position_observable
        self.altitude = altitude
        self.distance_threshold = distance_threshold
        self.local_setposition_publisher = local_setposition_publisher
        self.status = ActionState.WORKING
        self.commanded = False
        self.waypoint_published = False
        self.position_update = rx.empty().subscribe()
        self.waypoint = None

    def step(self):
        if not self.commanded:
            self.commanded = True

            def updatePosition(localposition):

                if not self.waypoint_published:
                    self.waypoint_published = True

                    self.waypoint = PoseStamped()
                    self.waypoint.pose.position.x = localposition.pose.position.x
                    self.waypoint.pose.position.y = localposition.pose.position.y
                    self.waypoint.pose.position.z = self.altitude
                    self.waypoint.pose.orientation.w = localposition.pose.orientation.w

                    self.local_setposition_publisher.publish(self.waypoint)

                if distance(self.waypoint.pose.position, localposition.pose.position) < self.distance_threshold:
                    self.status = ActionState.SUCCESS
                    self.stop()

            self.position_update = self.local_position_observable.subscribe(on_next = lambda position: updatePosition(position))

        return self.status

    def stop(self):
        self.position_update.dispose()
