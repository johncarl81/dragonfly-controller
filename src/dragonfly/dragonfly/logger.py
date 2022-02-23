#!/usr/bin/env python
import argparse
import sched
import time
from datetime import datetime, timedelta

import rclpy
from led import LED
from rclpy.qos import QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


class co2Logger:

    def __init__(self, id):
        self.node = None
        self.id = id

        self.position = None
        self.positionReceived = None
        self.co2Received = None
        self.led = LED()
        self.sincezero = datetime.now()
        self.zeroing = False
        self.s = sched.scheduler(time.time, time.sleep)
        self.time_offset = 0

    def validUpdate(self, inputTime):
        return inputTime is not None and datetime.now() - inputTime < timedelta(seconds=3)

    def updateStatus(self, position=None, co2=None, data=None):
        if position is not None:
            self.positionReceived = datetime.now()
        if co2 is not None:
            self.co2Received = datetime.now()
        if data is not None and (data.data.startswith('W') or data.data.startswith('Z')):
            self.sincezero = datetime.now()
        previous = self.zeroing
        self.zeroing = datetime.now() - self.sincezero < timedelta(seconds=10)
        if not self.zeroing == previous:
            print("Checking zero: {} {}".format(self.zeroing, previous))
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
        self.s.enter(1, 1, self.updateLED, ())

    def callback(self, data):
        self.position = data
        self.updateStatus(position=True)

    def getDate(self):
        return datetime.now() + timedelta(self.time_offset)

    def co2Callback(self, data):
        self.updateStatus(co2=True, data=data)
        if self.position is not None:
            print("{} co2: '{}' @ {} {} {}".format(self.getDate(),
                                                   data,
                                                   self.position.latitude,
                                                   self.position.longitude,
                                                   self.position.altitude))
        else:
            print("{} cos: '{}' @ -".format(self.getDate(), data))

    def logCallback(self, data):
        print("LOG: {}".format(data))

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rclpy.init(args=self.id)
        self.node = rclpy.create_node('gpslistener')

        # flight_computer_time = rclpy.wait_for_message(,
        # TimeReference)
        # flight_computer_time = self.node.create_subscription(TimeReference, "{}/mavros/global_position/global".format(self.id), qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))
        # @TODO fix me as the wait_for_message no longer exists at this time
        # self.time_offset = flight_computer_time.time_ref - datetime.now()
        self.time_offset = datetime.now()

        self.node.create_subscription(NavSatFix, "{}/mavros/global_position/global".format(self.id), self.callback,
                                      qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))
        self.node.create_subscription(String, "{}/co2".format(self.id), self.co2Callback, qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))
        self.node.create_subscription(String, "{}/log".format(self.id), self.logCallback, qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

        self.s.enter(1, 1, self.updateLED, ())
        self.s.run()

        # spin() simply keeps python from exiting until this node is stopped
        rclpy.spin(self.node)


def main():
    parser = argparse.ArgumentParser(description='Log the given drone\'s GPS And CO2.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    co2Logger(args.id).listener()


if __name__ == '__main__':
    main()
