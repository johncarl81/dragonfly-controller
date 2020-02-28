#!/usr/bin/env python
import rospy
import argparse
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import State

def land(id):

    rospy.init_node('land_service')
    rospy.wait_for_service("{}/mavros/cmd/land".format(id))

    land_service = rospy.ServiceProxy("{}/mavros/cmd/land".format(id), CommandTOL)

    print "Landing"
    print land_service(altitude = 0)

    def updateState(state):

        if not state.armed:
            print "Landed."
            print "State: ", state
            
            setmode_service = rospy.ServiceProxy("{}/mavros/set_mode".format(id), SetMode)
            print "Set Mode"
            print setmode_service(custom_mode = "STABALIZE")

            position_update.unregister()

    print "Waiting for landing..."
    position_update = rospy.Subscriber("{}/mavros/state".format(id), State, updateState)

    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Command a drone to land.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    land(args.id)