#! /usr/bin/env python
import rospy
import time
import argparse
from std_srvs.srv import Trigger, TriggerResponse
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State

class DragonflyCommand:

    def __init__(self, id):
        self.id = id

    def setmode(self, mode):
        print "Set Mode ", mode
        print self.setmode_service(custom_mode = mode)
        time.sleep(1)

    def arm(self):
        print "Arming"
        print self.arm_service(True)

        time.sleep(5)

    def disarm(self):
        print "Disarming"
        print self.arm_service(False)


    def armcommand(self, operation):
        print "Commanded to arm"

        self.setmode("STABILIZE")

        self.arm()

        self.disarm()

        return TriggerResponse(success=True, message="Commanded {} to arm.".format(self.id))

    def takeoff(self, operation):
        print "Commanded to takeoff"

        self.setmode("STABILIZE")

        self.arm()

        self.setmode("GUIDED")

        print "Take off"
        print self.takeoff_service(altitude = 3)

        return TriggerResponse(success=True, message="Commanded {} to takeoff.".format(self.id))

    def land(self, operation):
        print "Commanded to land"

        print "Landing"
        print self.land_service(altitude = 0)

        def updateState(state):

            if not state.armed:
                print "Landed."
                print "State: ", state

                self.setmode("STABILIZE")

                position_update.unregister()

        print "Waiting for landing..."
        position_update = rospy.Subscriber("{}/mavros/state".format(self.id), State, updateState)

        return TriggerResponse(success=True, message="Commanded {} to land.".format(self.id))

    def rtl(self, operation):
        print "Commanded to land"

        self.setmode("RTL")

        return TriggerResponse(success=True, message="Commanded {} to rtl.".format(self.id))

    def setup(self):
        rospy.init_node("{}_remote_service".format(self.id))

        rospy.wait_for_service("{}/mavros/set_mode".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/arming".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/takeoff".format(self.id))


        self.setmode_service = rospy.ServiceProxy("{}/mavros/set_mode".format(self.id), SetMode)
        self.arm_service = rospy.ServiceProxy("{}/mavros/cmd/arming".format(self.id), CommandBool)
        self.takeoff_service = rospy.ServiceProxy("{}/mavros/cmd/takeoff".format(self.id), CommandTOL)
        self.land_service = rospy.ServiceProxy("{}/mavros/cmd/land".format(self.id), CommandTOL)

        rospy.Service("/{}/command/arm".format(self.id), Trigger, self.armcommand)
        rospy.Service("/{}/command/takeoff".format(self.id), Trigger, self.takeoff)
        rospy.Service("/{}/command/land".format(self.id), Trigger, self.land)
        rospy.Service("/{}/command/rtl".format(self.id), Trigger, self.rtl)

        print "Setup complete"

        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Drone command service.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    command = DragonflyCommand(args.id)

    command.setup()

