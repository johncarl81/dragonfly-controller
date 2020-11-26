#! /usr/bin/env python
import math, rospy, rx, operator
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from ActionState import ActionState
from rx.subjects import Subject

class FlockingAction:

    def __init__(self, id, local_setvelocity_publisher, xoffset, yoffset, leader):
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.id = id
        self.xoffset = xoffset
        self.yoffset = yoffset
        self.leader = leader
        self.started = False

        self.flock_coordinates = {}
        self.leaderposition_subject = Subject()
        self.selfposition_subject = Subject()
        self.leadervelocity_subject = Subject()

        self.leaderposition_subject.zip(self.selfposition_subject, self.leadervelocity_subject, lambda a, b, c: [a,b,c]) \
                .subscribe(lambda value: self.printStuff(value))

    def printStuff(self, input):


        print "self: {}, {} target: {}, {}".format(input[0].pose.position.x, input[0].pose.position.y, input[1].pose.position.x, input[1].pose.position.y)

        # TODO: need to property orient between self and the lead - maybe using a TF?
        offsetx = input[0].pose.position.x - input[1].pose.position.x + self.xoffset
        offsety = input[0].pose.position.y - input[1].pose.position.y + self.yoffset
        # print input
        twist = TwistStamped()
        twist.twist.linear.x = input[2].twist.linear.x + (0.3 * offsetx)
        twist.twist.linear.y = input[2].twist.linear.y + (0.3 * offsety)

        print "Publishing velocity ({},{}) twist: {}".format(twist.twist.linear.x, twist.twist.linear.y, twist.twist.angular.z)
        self.local_setvelocity_publisher.publish(twist)


    def leaderposition_callback(self, pose):
        # print "Received pose"
        self.leaderposition_subject.on_next(pose)

    def selfposition_callback(self, pose):
        # print "Received pose"
        self.selfposition_subject.on_next(pose)

    def leadervelocity_callback(self, twist):
        # print "Received twist"
        self.leadervelocity_subject.on_next(twist)

    def flock_announce(self, name):
        if name.data not in self.flock_coordinates :
            flock_coordinate_subject = Subject()
            self.flock_coordinates[name.data] = flock_coordinate_subject

            def flock_coordiante_subject(position):
                print "name: {} position: {} {}".format(name.data, position.latitude, position.longitude)
                flock_coordinate_subject.on_next(position)

            rospy.Subscriber("{}/mavros/global_position/global".format(name.data), NavSatFix, flock_coordiante_subject)



    def step(self):
        if not self.started:
            self.started = True

            print "Subscribing..."

            rospy.Subscriber("{}/mavros/local_position/pose".format(self.leader), PoseStamped, self.leaderposition_callback)
            rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped, self.selfposition_callback)
            rospy.Subscriber("{}/mavros/local_position/velocity_local".format(self.leader), TwistStamped, self.leadervelocity_callback)
            # rospy.Subscriber("/dragonfly/announce", String, self.flock_announce)


        return ActionState.WORKING
