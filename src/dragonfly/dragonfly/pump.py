#!/usr/bin/env python3
import sys
import argparse
import time
import rclpy
import rx
import rx.operators as ops
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from dragonfly_messages.srv import Pump
from std_msgs.msg import String
from rx.scheduler import NewThreadScheduler


class BagInflateService(Node):
    def __init__(self, arg_id, sim=False):
        super().__init__('bag_inflate_service')
        self.id = arg_id
        self.sim = sim
        self.bag_gpio_pins = [16, 19, 20, 21]
        if not self.sim:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.bag_gpio_pins, GPIO.OUT)
        self.bag_full = [False] * len(self.bag_gpio_pins)
        self.inflate = self.create_service(Pump, f"/{self.id}/pump", self.bag_inflate_callback)
        # self.swap = self.create_service(BagSwap, "/{}/bagswap".format(self.id), self.bag_swap_callback)
        self.logPublisher = self.create_publisher(String, f"{self.id}/log", qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

    def bag_swap_callback(self, request, response):
        self.get_logger().info("Bags swapped")
        self.bag_full = [False] * len(self.bag_gpio_pins)
        return True

    def bag_inflate_callback(self, request, response):
        # @TODO add timestamp and id ect
        response.done = False
        self.get_logger().info(f"Bag {request.pump_num} inflate request received")
        if not self.bag_full[request.pump_num]:
            self.bag_full[request.pump_num] = True
            rx.just(request.pump_num).pipe(
                ops.observe_on(NewThreadScheduler()))\
                .subscribe(on_next=lambda pin: self.pump(pin))
            response.done = True
        else:
            self.get_logger().warn(f"Bag {request.pump_num} already inflated")
        return response

    def pump(self, pump_num):
        pin = self.bag_gpio_pins[pump_num]
        self.logPublisher.publish(String(data=f"Bag {pump_num} inflating..."))
        if not self.sim:
            GPIO.output(pin, 1)
        time.sleep(60)
        if not self.sim:
            GPIO.output(pin, 0)
        self.logPublisher.publish(String(data=f"Bag {pump_num} finished inflating."))


def main(args=None):
    parser = argparse.ArgumentParser(description='Sample Bag Collection Service')
    parser.add_argument('id', type=str, help='Name of the drone.')
    parser.add_argument('--sim', help='Is the sim running', action='store_true')
    parser.set_defaults(sim=False)
    args = parser.parse_args()
    print(f"STARTING PUMP {args.sim}")
    rclpy.init(args=sys.argv)
    bag_inflate_service = BagInflateService(args.id, args.sim)
    rclpy.spin(bag_inflate_service)
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == '__main__':
    main(args=sys.argv)
