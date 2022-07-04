#!/usr/bin/env python

import argparse

import rclpy
import serial
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from rclpy.qos import HistoryPolicy


def publishco2(id):
    rclpy.init()  # args=id
    node = rclpy.create_node('talker')
    node.get_logger().info("publishing co2 readings on {}/co2".format(id))
    pub = node.create_publisher(String, "{}/co2".format(id), qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))
    rate = node.create_rate(10)
    while rclpy.ok():
        try:
            with serial.Serial("/dev/ttysba5", baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                               stopbits=serial.STOPBITS_ONE) as port:
                node.get_logger().info('Connected to /dev/ttysba5')
                # Publish on demand
                port.write(str.encode('!')) 
                # Configure to 2 decimal places
                port.write('C2\r')
                while rclpy.ok():
                    port.write('M')
                    hello_str = port.readline()
                    
                    #@TODO split and put into 
                    #/home/carter/dfly/dragonfly-controller/src/dragonfly_messages/msg/CO2.msg
                    
                    
                    pub.publish(hello_str)
                    rate.sleep()
        except serial.SerialException as ex:
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(description='Starts ROS publisher for CO2 sensor.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publishco2(args.id)


if __name__ == '__main__':
    main()
