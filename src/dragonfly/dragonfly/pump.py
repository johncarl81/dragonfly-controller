#!/usr/bin/env python3
import sys
import argparse
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from dragonfly_messages.srv import Pump
import RPi.GPIO as GPIO

class BagInflateService(Node):
  def __init__(self, arg_id):
    super().__init__('bag_inflate_service')
    self.id = arg_id
    self.bag_gpio_pins = [16, 19, 20, 21]
    self.bag_full = [False] * len(self.bag_gpio_pins)
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(self.bag_gpio_pins, GPIO.OUT)
    self.inflate = self.create_service(Pump, "/{}/pump".format(self.id), self.bag_inflate_callback)
    #self.swap = self.create_service(BagSwap, "/{}/bagswap".format(self.id), self.bag_swap_callback)

  def bag_swap_callback(self, request):
    self.get_logger().info("Bags swapped")
    self.bag_full = [False] * len(self.bag_gpio_pins)
    return Bool(True)

  def bag_inflate_callback(self, request):
    # @TODO add timestamp and id ect
    self.get_logger().info("Bag inflate request received")
    if not self.bag_full[request.pump_num]:
      self.bag_full[request.pump_num] = True
      GPIO.output(self.bag_gpio_pins[request.pump_num], 1)
      time.sleep(60)
      GPIO.output(self.bag_gpio_pins[request.pump_num], 0)
      return Bool(True)
    self.get_logger().warn("Bag already inflated")
    return Bool(False)


def main(args=None):
  parser = argparse.ArgumentParser(description='Sample Bag Collection Service')
  parser.add_argument('id', type=str, help='Name of the drone.')
  args = parser.parse_args()
  rclpy.init(args=sys.argv)
  bag_inflate_service = BagInflateService(args.id)
  rclpy.spin(bag_inflate_service)
  rclpy.shutdown()
  GPIO.cleanup()


if __name__ == '__main__':
  main(args=sys.argv)
