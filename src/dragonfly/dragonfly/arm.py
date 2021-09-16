#!/usr/bin/env python
import argparse
import rospy
import time
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode


def arm(id):
    rospy.init_node('arm_test_service')
    rospy.wait_for_service("{}/mavros/set_mode".format(id))
    rospy.wait_for_service("{}/mavros/cmd/arming".format(id))

    setmode_service = rospy.ServiceProxy("{}/mavros/set_mode".format(id), SetMode)
    arm_service = rospy.ServiceProxy("{}/mavros/cmd/arming".format(id), CommandBool)

    print("Setup complete")

    print("Set Mode")
    print(setmode_service(custom_mode="STABILIZE"))

    time.sleep(1)

    print("Arming")
    print(arm_service(True))

    time.sleep(5)

    print("Disarming")
    print(arm_service(False))

    print("Commanded")


if __name__ == '__main__':
    # Get RGB colors from command line arguments.
    parser = argparse.ArgumentParser(description='Arm a drone.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    arm(args.id)
