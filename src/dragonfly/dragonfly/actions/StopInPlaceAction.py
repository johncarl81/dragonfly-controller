#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped
from .ActionState import ActionState
from std_msgs.msg import String

class StopInPlaceAction:

    def __init__(self, id, log_publisher, local_setposition_publisher, node):
        self.id = id
        self.log_publisher = log_publisher
        self.local_setposition_publisher = local_setposition_publisher
        self.status = ActionState.WORKING
        self.commanded = False
        self.position_update = None
        self.node = node

    def step(self):
        if not self.commanded:
            self.commanded = True

            def updatePosition(localposition):
                self.local_setposition_publisher.publish(localposition)
                self.status = ActionState.SUCCESS

                self.stop()

                print("Stop in place")
                self.log_publisher.publish(String(data="Stopped"))

            self.node.create_subscription(PoseStamped, "{}/mavros/local_position/pose".format(self.id), updatePosition,
                                          10)

        return self.status

    def stop(self):
        if self.position_update is not None:
            self.position_update.destroy()
            self.position_update = None
