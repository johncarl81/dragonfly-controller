#! /usr/bin/env python
import rospy
import argparse
from std_srvs.srv import Trigger, TriggerResponse

def command(operation):
    print "Command: {}".format(operation)

    return TriggerResponse(success=True, message="hello, commanded")

def setupService(id):
    rospy.init_node('dragonfly_remote_service')
    service = rospy.Service("/{}/dragonfly_command".format(id), Trigger, command)

    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Drone command service.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    setupService(args.id)

