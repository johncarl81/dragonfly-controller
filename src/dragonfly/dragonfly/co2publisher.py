#!/usr/bin/env python3

import argparse
import rclpy
import serial
import sys
import re

from rclpy.qos import QoSProfile
from rclpy.qos import HistoryPolicy
from dragonfly_messages.msg import CO2

class CO2Publisher:

  def __init__(self, id, node):
    self.id = id
    self.zeroing = False
    self.zeroing_current_count = 0
    self.init_zeroing_count = 2
    self.logger = node.get_logger()
    self.pub = node.create_publisher(CO2, f"{id}/co2",
                                     qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))
    self.sba5_polling_rate = node.create_rate(10)
    self.error_retry_rate = node.create_rate(1)

  def parse_sba5_message(self, sba5_str):
    self.logger.debug("co2_publisher:" + sba5_str)

    reading = None
    sba5_data = re.split(" |,", sba5_str.strip())
    if sba5_data[0] == 'M' and len(sba5_data) == 11:
      # M 50885 48094 500.96 55.0 0.0 0.0 829 55.0 55.0 00
      reading = CO2(ppm=float(sba5_data[3]),
          average_temp=float(sba5_data[4]),
          humidity=float(sba5_data[5]),
          humidity_sensor_temp=float(sba5_data[6]),
          atmospheric_pressure=int(sba5_data[7]),
          detector_temp=float(sba5_data[8]),
          source_temp=float(sba5_data[9]),
          status=int(sba5_data[10]))
    elif sba5_data[0] == 'Z' and len(sba5_data) == 4:
      # Z,22 of 25\r\n
      reading = CO2(zeroing=True,
          zeroing_index=int(sba5_data[1]),
          zeroing_count=int(sba5_data[3]))
    elif sba5_data[0] == 'W' and len(sba5_data) == 2:
      # W,43.1
      reading = CO2(warming=True,
          average_temp=float(sba5_data[1]))
    else:
      self.logger.error(f"Unable to parse SBA-5 message: {sba5_str}")

    return reading
  def publish(self):
    self.logger.info(f"publishing co2 readings on {self.id}/co2")

    while rclpy.ok():
      try:
        with serial.Serial("/dev/ttysba5", baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE) as port:
          self.logger.info('Connected to /dev/ttysba5')
          port.write(str.encode('!'))  # measurement display off
          port.write(str.encode('C2\r'))  # Configure to 2 decimal places
          port.write(str.encode('P1\r'))  # turn on pump
          port.write(str.encode('A2\r')) #Time [minutes] between zero operations
          port.write(str.encode('Z')) # Perform a zero operation.
          while rclpy.ok():
            if not self.zeroing:
              port.write(str.encode('M'))  # request measurement

            sba5_data = self.parse_sba5_message(str(port.readline(), 'UTF-8'))

            if sba5_data is not None:
              if sba5_data.status == CO2.NO_ERROR:
                pre_zeroing = self.zeroing
                self.zeroing = sba5_data.warming or (sba5_data.zeroing and sba5_data.zeroing_index < sba5_data.zeroing_count)
                if (not pre_zeroing == self.zeroing) and not self.zeroing:
                  self.zeroing_current_count = self.zeroing_current_count + 1
                  if self.init_zeroing_count == self.zeroing_current_count:
                    port.write(str.encode('A120\r')) #Time [minutes] between zero operations
                self.pub.publish(sba5_data)
            self.sba5_polling_rate.sleep()

      except serial.SerialException as ex:
        self.logger.warn("SerialException: " + str(ex))
        self.error_retry_rate.sleep()


def main():
  parser = argparse.ArgumentParser(description='Starts ROS publisher for CO2 sensor.')
  parser.add_argument('id', type=str, help='Name of the drone.')
  args = parser.parse_args()

  rclpy.init(args=sys.argv)
  node = rclpy.create_node('co2_publisher')

  thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
  thread.start()

  publisher = CO2Publisher(args.id, node)
  publisher.publish()

if __name__ == '__main__':
  main()
