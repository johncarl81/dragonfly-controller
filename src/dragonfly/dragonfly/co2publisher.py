#!/usr/bin/env python3

import argparse
import time  # temporary measure until rate's timer works
import rclpy
import serial
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rclpy.qos import HistoryPolicy
from dragonfly_messages.msg import CO2


def publishco2(args):
  rclpy.init()  # args=id
  node = rclpy.create_node('co2_publisher')
  node.get_logger().info("publishing co2 readings on {}/co2".format(args))
  pub = node.create_publisher(String, "{}/co2".format(args),
                              qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

  sba5_pub = node.create_publisher(CO2, "{}/SBA5".format(args),
                                   qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))
  # Using time's sleep for now until rate timer works correctly
  # rate = node.create_rate(10, node.get_clock())
  while rclpy.ok():
    try:
      with serial.Serial("/dev/ttysba5", baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                         stopbits=serial.STOPBITS_ONE) as port:
        node.get_logger().info('Connected to /dev/ttysba5')
        port.write(str.encode('!'))  # measurement display off
        port.write(str.encode('C2\r'))  # Configure to 2 decimal places
        port.write(str.encode('P1\r'))  # turn on pump
        while rclpy.ok():
          port.write(str.encode('M'))  # request measurement
          sba5_str = port.readline()
          if sba5_str.startswith(b'M '):  # b'M 54981 52166 0.00 54.0 0.0 0.0 849 54.0 52.8 08\r\n'
            node.get_logger().info("co2_publisher:" + str(sba5_str))
            # @TODO might remove this log or throttle it so it does not fill up the logs
            sba5_data = str(sba5_str).replace('\\r\\n\'', '').split(" ")
            print("sba5_data: ")
            print(sba5_data)
            if len(sba5_data) > 9 and  sba5_data[10].isdigit():
              status = int(sba5_data[10])
              if status == CO2.NO_ERROR:
                sba5_pub.publish(CO2(zero_count=int(sba5_data[1]), count=int(sba5_data[2]), ppm=float(sba5_data[3]),
                                     average_temp=float(sba5_data[4]),
                                     humidity=float(sba5_data[5]), humidity_sensor_temp=float(sba5_data[6]),
                                     atmospheric_pressure=int(sba5_data[7]), detector_temp=float(sba5_data[8]),
                                     source_temp=float(sba5_data[9]),
                                     status=status))
              else:
                # @TODO print usefull msg CO2.LOW_COUNT, TOO_COLD,TOO_HOT, LOW_ALARM, HIGH_HUMIDITY, LOW_VOLTAGE like
                #  the var name
                node.get_logger().error("co2_publisher: sba5 error: #" + str(int(status)))
            else:
              node.get_logger().error("co2_publisher: sba5 error: #?")
            pub.publish(sba5_str)
            # Using time's sleep for now until rate timer works correctly
            # rate.sleep()
            time.sleep(.1)

    except serial.SerialException as ex:
      node.get_logger().warn("SerialException: " + str(ex))
      # Using time's sleep for now until rate timer works correctly
      # rate.sleep()
      time.sleep(.1)


def main():
  parser = argparse.ArgumentParser(description='Starts ROS publisher for CO2 sensor.')
  parser.add_argument('id', type=str, help='Name of the drone.')
  args = parser.parse_args()

  publishco2(args.id)


if __name__ == '__main__':
  main()
