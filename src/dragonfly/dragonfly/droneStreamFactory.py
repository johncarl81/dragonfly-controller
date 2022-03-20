#!/usr/bin/env python
from rx.subject import Subject

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSHistoryPolicy, HistoryPolicy, ReliabilityPolicy

class DroneStream:

    def __init__(self, name, node):
        self.name = name
        self.node = node

        self.position_subject = Subject()
        self.position_subject_init = False
        self.co2_subject = Subject()
        self.co2_subject_init = False
        self.velocity_subject = Subject()
        self.velocity_subject_init = False

    def get_position(self):
        if not self.position_subject_init:
            self.node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(self.name),
                                          lambda position: self.position_subject.on_next(position),
                                          qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.position_subject_init = True

        return self.position_subject

    def get_co2(self):
        if not self.co2_subject_init:
            self.node.create_subscription(String, "{}/co2".format(self.name), lambda value: self.co2_subject.on_next(value),
                                          qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.co2_subject_init = True

        return self.co2_subject

    def get_velocity(self):
        if not self.velocity_subject_init:
            self.node.create_subscription(TwistStamped, "{}/mavros/local_position/velocity_local".format(self.name),
                                          lambda value: self.velocity_subject.on_next(value),
                                          qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.velocity_subject_init = True

        return self.velocity_subject


class DroneStreamFactory:

    def __init__(self, node):
        self.node = node

        self.drones = {}

    def get_drone(self, name):
        if name not in self.drones.keys():
            self.drones[name] = DroneStream(name, self.node)

        return self.drones[name]