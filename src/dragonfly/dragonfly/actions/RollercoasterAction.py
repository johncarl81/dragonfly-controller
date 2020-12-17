#! /usr/bin/env python3
import math, time
from geometry_msgs.msg import TwistStamped
from .ActionState import ActionState

class RollercoasterAction:


    def __init__(self, local_setvelocity_publisher):
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.start = None

    def rollercoaster(self, t):
        velocity = 0.75
        return [velocity * math.sin((2 * math.pi / 50) * t),
                velocity * math.cos((2 * math.pi / 50) * t) * 0.75,
                velocity * math.sin((6 * math.pi / 50) * t)]

    def step(self):
        if self.start is None:
            self.start = time.time()

        deltatime = time.time() - self.start

        velocity_vector = self.rollercoaster(deltatime)

        twist = TwistStamped()

        twist.twist.linear.x += velocity_vector[0]
        twist.twist.linear.y += velocity_vector[1]
        twist.twist.linear.z += velocity_vector[2]

        self.local_setvelocity_publisher.publish(twist)

        return ActionState.WORKING

    def stop(self):
        pass
