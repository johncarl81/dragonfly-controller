#!/usr/bin/env python
import argparse
import rospy
import time
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode


def takeoff(id):
    rospy.init_node('takeoff_service')
    rospy.wait_for_service("{}/mavros/set_mode".format(id))
    rospy.wait_for_service("{}/mavros/cmd/arming".format(id))
    rospy.wait_for_service("{}/mavros/cmd/takeoff".format(id))

    setmode_service = rospy.ServiceProxy("{}/mavros/set_mode".format(id), SetMode)
    arm_service = rospy.ServiceProxy("{}/mavros/cmd/arming".format(id), CommandBool)
    takeoff_service = rospy.ServiceProxy("{}/mavros/cmd/takeoff".format(id), CommandTOL)
    print("Setup complete")

    print("Set Mode")
    print(setmode_service(custom_mode="STABILIZE"))

    time.sleep(1)

    print("Arming")
    print(arm_service(True))

    time.sleep(5)

    print("Change to Guided")

    print(setmode_service(custom_mode="GUIDED"))

    print("Take off")
    print(takeoff_service(altitude=3))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Command a drone to takeoff.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    takeoff(args.id)
