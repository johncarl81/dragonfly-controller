#! /usr/bin/env python
import math, rospy, rx, operator
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from ActionState import ActionState
from rx.subjects import Subject
from rx.core import Observable

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
        self.flock_repulsion = Subject()

        formation_position_attraction = Observable.zip(self.leaderposition_subject, self.selfposition_subject,
                                                       lambda leaderposition, selfposition: self.formation_position(leaderposition, selfposition))

        leaderVelocity = self.leadervelocity_subject \
            .map(lambda twist: self.format_velocities(twist))

        Observable.zip_array(leaderVelocity, formation_position_attraction, self.flock_repulsion) \
            .subscribe(lambda vectors: self.navigate(vectors))

        self.flockSubscription = Observable.empty().subscribe()

    def navigate(self, input):

        twist = TwistStamped()

        print "Magnitudes: {}".format([self.magnitude(v) for v in input])

        for vector in input:
            twist.twist.linear.x += vector[0]
            twist.twist.linear.y += vector[1]

        self.local_setvelocity_publisher.publish(twist)

    def differenceInMeters(self, one, two):
        earthCircumference = 40008000
        return [
            ((one.longitude - two.longitude) * (earthCircumference / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (earthCircumference / 360))
        ]

    def formation_position(self, leaderPosition, selfPosition):
        difference = self.differenceInMeters(leaderPosition, selfPosition)

        return [
            (0.3 * (difference[0] + self.xoffset)),
            (0.3 * (difference[1] + self.yoffset))
        ]

    def format_velocities(self, twist):
        return [
            twist.twist.linear.x,
            twist.twist.linear.y
        ]

    def magnitude(self, vector):
        return math.sqrt((vector[0] * vector[0]) + (vector[1] * vector[1]))

    def repulsion_vector(self, positions):
        equilibrium_distance = 3

        vector = [0,0]
        if len(positions) > 1:
            self_position = positions[0]
            for position in positions[1:]:
                difference = self.differenceInMeters(self_position, position)
                difference_magnitude = self.magnitude(difference)
                if difference_magnitude < equilibrium_distance:
                    vector[0] += (equilibrium_distance - difference_magnitude) * difference[0] / difference_magnitude
                    vector[1] += (equilibrium_distance - difference_magnitude) * difference[1] / difference_magnitude

        return vector

    def flock_announce_callback(self, nameString):
        self.flock_announce(nameString.data)

    def flock_announce(self, name):
        if name != self.id and name not in self.flock_coordinates :
            print "Registering flock member: {}".format(name)
            flock_coordinate_subject = Subject()
            self.flock_coordinates[name] = flock_coordinate_subject

            def flock_coordiante_subject(position):
                # print "name: {} position: {} {}".format(name, position.latitude, position.longitude)
                flock_coordinate_subject.on_next(position)

            rospy.Subscriber("{}/mavros/global_position/global".format(name), NavSatFix, flock_coordiante_subject)

            self.flockSubscription.dispose()

            flock_coordinate_subject_list = [self.selfposition_subject]
            flock_coordinate_subject_list.extend(self.flock_coordinates.values())



            self.flockSubscription = Observable.zip_array(*flock_coordinate_subject_list) \
                    .map(lambda positions: self.repulsion_vector(positions)) \
                    .subscribe(on_next= lambda v: self.flock_repulsion.on_next(v))

    def step(self):
        if not self.started:
            self.started = True

            print "Subscribing..."

            rospy.Subscriber("{}/mavros/global_position/global".format(self.leader), NavSatFix, lambda position: self.leaderposition_subject.on_next(position))
            rospy.Subscriber("{}/mavros/global_position/global".format(self.id), NavSatFix, lambda position: self.selfposition_subject.on_next(position))
            rospy.Subscriber("{}/mavros/local_position/velocity_local".format(self.leader), TwistStamped, lambda twist: self.leadervelocity_subject.on_next(twist))
            rospy.Subscriber("/dragonfly/announce", String, self.flock_announce_callback)


        return ActionState.WORKING
