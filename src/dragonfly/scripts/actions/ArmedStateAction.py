#! /usr/bin/env python
import rospy
from mavros_msgs.msg import State
from actions.ActionQueue import ActionQueue

class ArmedStateAction:

    def __init__(self, id):
        self.id = id
        self.armedQueue = ActionQueue()
        self.notarmedQueue = ActionQueue()

    def step(self):
        print "Verifying disarmed..."
        def updateState(state):

            if not state.armed:
                while self.armedQueue.step():
                    continue
                # print "Commanded to takeoff"
                #
                # self.setmode("STABILIZE")
                #
                # self.arm()
                #
                # self.setmode("GUIDED")
                #
                # print "Take off"
                # print self.takeoff_service(altitude = 3)
            else:
                while self.notarmedQueue.step():
                    continue

            disabled_update.unregister()

        disabled_update = rospy.Subscriber("{}/mavros/state".format(self.id), State, updateState)

        return True

    def armed(self):
        return self.armedQueue

    def notarmed(self):
        return self.notarmedQueue