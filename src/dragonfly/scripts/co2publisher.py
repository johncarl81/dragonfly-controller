#!/usr/bin/env python

import argparse

import rospy
import serial
from dragonfly_messages.msg import CO2

class CO2Publisher:

    def __init__(self, id):
        self.id = id
        self.zeroing = False

        rospy.loginfo("publishing co2 readings on {}/co2".format(id))
        self.pub = rospy.Publisher("{}/co2".format(id), CO2, queue_size=10)
        rospy.init_node('talker', anonymous=True)

    def parse_command(self, input):
        reading = None
        parts = input.strip().split(" ")
        if parts[0] == 'M' and len(parts) == 11:
            # M 50885 48094 500.96 55.0 0.0 0.0 829 55.0 55.0 00
            reading = CO2()
            reading.ppm = float(parts[3])
            reading.sensor_temp = float(parts[4])
            reading.humidity = float(parts[5])
            reading.humidity_sensor_temp = float(parts[6])
            reading.atmospheric_pressure = float(parts[7])
            reading.detector_temp = float(parts[8])
            reading.source_temp = float(parts[9])
            reading.status = int(parts[10])
        elif parts[0].startswith('Z') and len(parts) == 3:
            # Z,22 of 25\r\n
            zeroing_parts = parts[0].split(',')
            reading = CO2()
            reading.zeroing = True
            reading.zeroing_index = int(zeroing_parts[1])
            reading.zeroing_count = int(parts[2])
        elif parts[0].startswith('W') and len(parts) == 1:
            # W,43.1
            warming_parts = parts[0].split(',')
            reading = CO2()
            reading.warming = True
            reading.sensor_temp = float(warming_parts[1])

        return reading

    def publish(self):

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                with serial.Serial("/dev/ttysba5", baudrate=19200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                   stopbits=serial.STOPBITS_ONE) as port:
                    rospy.loginfo('Connected to /dev/ttysba5')
                    # Publish on demand
                    port.write('!')
                    # Configure to 2 decimal places
                    port.write('C2\r')
                    while not rospy.is_shutdown():
                        if not self.zeroing:
                            port.write('M')

                        value = self.parse_command(port.readline())

                        if value is not None:
                            self.zeroing = value.warming or (value.zeroing and value.zeroing_index < value.zeroing_count)
                            self.pub.publish(value)
                        rate.sleep()
            except serial.SerialException as ex:
                rospy.sleep(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Starts ROS publisher for CO2 sensor.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    publisher = CO2Publisher(args.id)

    publisher.publish()
