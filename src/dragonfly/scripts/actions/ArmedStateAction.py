#! /usr/bin/env python
import rospy
from mavros_msgs.msg import State
from actions.ActionQueue import ActionQueue
from ActionState import ActionState

class ArmedStateAction:

    def __init__(self, id):
        self.id = id
        self.armedQueue = ActionQueue()
        self.notarmedQueue = ActionQueue()
        self.status = ActionState.WORKING
        self.commanded = False

    def step(self):
        if not self.commanded:
            print "Verifying disarmed..."
            self.commanded = True
            disabled_update = None

            def updateState(state):

                if state.armed:
                    print "Is already armed, failed"
                    self.status = ActionState.FAILURE
                else:
                    print "Is not armed, continue"
                    self.status = ActionState.SUCCESS

                if not disabled_update is None:
                    disabled_update.unregister()

            disabled_update = rospy.Subscriber("{}/mavros/state".format(self.id), State, updateState)

        return self.status