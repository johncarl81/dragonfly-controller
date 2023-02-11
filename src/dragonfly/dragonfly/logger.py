#!/usr/bin/env python3
import argparse
import sys
import rx
import rclpy

from datetime import datetime, timedelta
from rclpy.qos import ReliabilityPolicy
from .led import LED
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix, TimeReference
from std_msgs.msg import String
from dragonfly_messages.msg import CO2


class co2Logger:

    def __init__(self, id):
        self.node = None
        self.id = id
        self.position = None
        self.positionReceived = None
        self.co2Received = None
        self.led = LED()
        self.zeroing = False
        self.time_offset = timedelta(seconds=0)

    def validUpdate(self, inputTime):
        return inputTime is not None and self.getDate() - inputTime < timedelta(seconds=3)

    def updateStatus(self, position=None, co2=None, data=None):
        if position is not None:
            self.positionReceived = self.getDate()
        if co2 is not None:
            self.co2Received = self.getDate()
        previous = self.zeroing
        if data is not None:
            self.zeroing = data.warming or data.zeroing
        if self.zeroing and not previous:
            self.led.blink()
        elif not self.zeroing and previous:
            self.led.solid()

    def updateLED(self):
        validPosition = self.validUpdate(self.positionReceived)
        validCo2 = self.validUpdate(self.co2Received)
        self.led.setColor([255 if validPosition and not validCo2 else 0,
                           255 if validPosition and validCo2 else 0,
                           255 if not validPosition and validCo2 else 0])

    def callback(self, data):
        self.position = data
        self.updateStatus(position=True)

    def timeCallback(self, data):
        self.time_offset = datetime.fromtimestamp(data.time_ref.sec) - datetime.now()

    def getDate(self):
        return datetime.now() + self.time_offset

    def co2Callback(self, data):
        self.updateStatus(co2=True, data=data)
        if self.position is not None:
            print(
                f"{self.getDate()} co2: {data.ppm} {data.average_temp} {data.humidity} {data.humidity_sensor_temp} {data.atmospheric_pressure} {data.detector_temp} {data.source_temp} {data.status} @ {self.position.latitude} {self.position.longitude} {self.position.altitude}")
        else:
            print(
                f"{self.getDate()} co2: {data.ppm} {data.average_temp} {data.humidity} {data.humidity_sensor_temp} {data.atmospheric_pressure} {data.detector_temp} {data.source_temp} {data.status} @ -")

    def logCallback(self, data):
        print(f"{self.getDate()} LOG: {data}")

    def listener(self):

        rclpy.init(args=sys.argv)
        self.node = rclpy.create_node('gpslistener')

        self.node.create_subscription(NavSatFix, f"/{self.id}/mavros/global_position/global", self.callback,
                                      qos_profile=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
        self.node.create_subscription(TimeReference, f"/{self.id}/mavros/time_reference", self.timeCallback,
                                      qos_profile=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
        self.node.create_subscription(CO2, f"/{self.id}/co2", self.co2Callback, qos_profile=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))
        self.node.create_subscription(String, f"/{self.id}/log", self.logCallback, qos_profile=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10))

        rx.interval(1).subscribe(
            on_next=lambda time: self.updateLED())

        rclpy.spin(self.node)


def main():
    parser = argparse.ArgumentParser(description='Log the given drone\'s GPS And CO2.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    co2Logger(args.id).listener()


if __name__ == '__main__':
    main()
