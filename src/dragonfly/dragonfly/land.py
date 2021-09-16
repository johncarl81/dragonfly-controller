#!/usr/bin/env python
import argparse

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode


def land(id):
    rospy.init_node('land_service')
    rospy.wait_for_service("{}/mavros/cmd/land".format(id))

    land_service = rospy.ServiceProxy("{}/mavros/cmd/land".format(id), CommandTOL)

    print("Landing")
    print(land_service(altitude=0))

    def updateState(state):
        if not state.armed:
            print("Landed.")
            print("State: {}".format(state))
            setmode_service = rospy.ServiceProxy("{}/mavros/set_mode".format(id), SetMode)
            print("Set Mode")
            print(setmode_service(custom_mode="STABILIZE"))

            position_update.destroy()

    print("Waiting for landing...")
    position_update = rospy.Subscriber("{}/mavros/state".format(id), State, updateState)

    rospy.spin()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Command a drone to land.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    land(args.id)
