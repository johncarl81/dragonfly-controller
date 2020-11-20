#! /usr/bin/env python
import rospy, time, argparse, math, std_msgs, pulp

from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State, Waypoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point
from dragonfly_messages.srv import *
from waypointUtil import *
from actions.DisarmAction import DisarmAction
from actions.ArmAction import ArmAction
from actions.ActionQueue import ActionQueue
from actions.ModeAction import ModeAction
from actions.SleepAction import SleepAction
from actions.TakeoffAction import TakeoffAction
from actions.PrintAction import PrintAction
from actions.ArmedStateAction import ArmedStateAction
from actions.WaypointAction import WaypointAction
from actions.LogAction import LogAction

class DragonflyCommand:

    def __init__(self, id):
        self.zeroing = False
        self.canceled = False
        self.sincezero = 0
        self.id = id
        self.actionqueue = ActionQueue()

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

    def hello(self, command):
        print "hello"
        return EmptyResponse()

    def armcommand(self, operation):
        print "Commanded to arm"

        self.actionqueue.push(ModeAction(self.setmode_service, "STABILIZE"))
        self.actionqueue.push(ArmAction(self.arm_service))
        self.actionqueue.push(SleepAction(5))
        self.actionqueue.push(DisarmAction(self.arm_service))

        return EmptyResponse()

    def takeoff(self, operation):

        armedStateAction = ArmedStateAction(self.id)

        armedStateAction.armed()\
            .push(ModeAction(self.setmode_service, "STABILIZE"))\
            .push(ArmAction(self.arm_service))\
            .push(SleepAction(5))\
            .push(ModeAction(self.setmode_service, "GUIDED"))\
            .push(TakeoffAction(self.takeoff_service, 3))

        armedStateAction.notarmed()\
            .push(PrintAction("Takeoff aborted, {} is armed".format(self.id)))

        self.actionqueue.push(armedStateAction)

        return EmptyResponse()

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

        return EmptyResponse()

    def rtl(self, operation):
        print "Commanded to land"

        self.setmode("RTL")

        return EmptyResponse()

    def goto(self, operation):
        print "Commanded to goto"

        def updatePosition(position):
            position_update.unregister()
            print "Position: ", position.latitude, " ", position.longitude

            rospy.rostime.wallsleep(0.5)

            goalPos = PoseStamped()
            goalPos.pose.position.x = 10
            goalPos.pose.position.y = 0
            goalPos.pose.position.z = self.altitude

            print "Going to: ", goalPos.pose.position
            print self.local_setposition_publisher.publish(goalPos)

            rospy.rostime.wallsleep(10)

            goalPos = PoseStamped()
            goalPos.pose.position.x = 0
            goalPos.pose.position.y = 0
            goalPos.pose.position.z = self.altitude

            print "Going to: ", goalPos.pose.position
            print self.local_setposition_publisher.publish(goalPos)

        position_update = rospy.Subscriber("{}/mavros/global_position/global".format(self.id), NavSatFix, updatePosition)

        return EmptyResponse()

    def build_ddsa_waypoints(self, walk, stacks, loops, radius, steplength, altitude):
        ddsaWaypoints = build3DDDSAWaypoints(Span(walk), stacks, 1, 0, loops, radius, steplength)

        localWaypoints = []
        for localwaypoint in ddsaWaypoints:
            localWaypoints.append(createWaypoint(self.localposition.x + localwaypoint.x, self.localposition.y + localwaypoint.y, altitude + localwaypoint.z))

        return localWaypoints


    def build_ddsa(self, operation):
        ddsaWaypoints = self.build_ddsa_waypoints(operation.walk, operation.stacks, operation.loops, operation.radius, operation.steplength, operation.altitude)

        waypoints = []
        for localwaypoint in ddsaWaypoints:
            waypoints.append(createLatLon(localwaypoint.pose.position, self.localposition, self.position))

        return DDSAWaypointsResponse(waypoints=waypoints)

    def ddsa(self, operation):
        print "Commanded to ddsa"
        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Started"))
        self.actionqueue.push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        waypoints = self.build_ddsa_waypoints(operation.walk, operation.stacks, operation.loops, operation.radius, operation.steplength, operation.altitude)

        self.runWaypoints(waypoints, operation.waittime, operation.distanceThreshold)

        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Finished"))

        return DDSAResponse(success=True, message="Commanded {} to DDSA.".format(self.id))

    def build_lawnmower_waypoints(self, walkBoundary, boundary, walk, altitude, stacks, steplength):
        lawnmowerLocalWaypoints = []

        if walkBoundary:
            wrappedGPSBoundary = []
            wrappedGPSBoundary.extend(boundary)
            wrappedGPSBoundary.append(boundary[0])

            for waypoint in wrappedGPSBoundary:
                lawnmowerLocalWaypoints.append(buildRelativeWaypoint(self.localposition, self.position, waypoint, altitude))

        lawnmowerLocalWaypoints.extend(build3DLawnmowerWaypoints(Span(walk), altitude, self.localposition, self.position, stacks, boundary, steplength))

        return lawnmowerLocalWaypoints

    def build_lawnmower(self, operation):

        waypoints = []
        for lawnmowerLocalWaypoint in self.build_lawnmower_waypoints(operation.walkBoundary, operation.boundary, operation.walk, operation.altitude, operation.stacks, operation.steplength):
            waypoints.append(createLatLon(lawnmowerLocalWaypoint.pose.position, self.localposition, self.position))

        return LawnmowerWaypointsResponse(waypoints=waypoints)

    def lawnmower(self, operation):
        print "Commanded to lawnmower"
        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Started"))
        self.actionqueue.push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        print "Position: {} {} {}".format(self.localposition.x, self.localposition.y, self.localposition.z)

        waypoints = self.build_lawnmower_waypoints(operation.walkBoundary, operation.boundary, operation.walk, operation.altitude, operation.stacks, operation.steplength)

        self.runWaypoints(waypoints, operation.waittime, operation.distanceThreshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Finished"))

        return LawnmowerResponse(success=True, message="Commanded {} to lawnmower.".format(self.id))

    def navigate(self, operation):
        print "Commanded to navigate"
        self.actionqueue.push(LogAction(self.logPublisher, "Navigation started"))
        self.actionqueue.push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        print "{} {}".format(self.localposition.z, self.position.altitude)

        localWaypoints = []
        for waypoint in operation.waypoints:
            print "{} {} {}".format(self.localposition.z, self.position.altitude, waypoint.relativeAltitude)
            localWaypoints.append(buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relativeAltitude))

        self.runWaypoints(localWaypoints, operation.waittime, operation.distanceThreshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Navigation Finished"))

        return NavigationResponse(success=True, message="Commanded {} to navigate.".format(self.id))

    def runWaypoints(self, waypoints, waittime, distanceThreshold):

        i = 0
        for waypoint in waypoints:
            self.actionqueue.push(WaypointAction(self.id, self.local_setposition_publisher, waypoint, waittime, distanceThreshold))

        return EmptyResponse()

    def position(self, data):
        # print data
        self.position = data

    def localposition(self, data):
        self.localposition = data.pose.position

    def co2Callback(self, data):
        self.sincezero = self.sincezero + 1
        if data.data.startswith('W') or data.data.startswith('Z'):
            self.sincezero = 0
        previous = self.zeroing
        self.zeroing = self.sincezero < 6
        if self.zeroing and not previous:
            self.logPublisher.publish('Zeroing')
        elif not self.zeroing and previous:
            self.logPublisher.publish('Finished zeroing')

    def cancel(self, data):
        self.canceled = True

    def loop(self):
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            self.actionqueue.step()
            # rospy.sleep(1)

    def setup(self):
        rospy.init_node("{}_remote_service".format(self.id), anonymous=True)

        rospy.wait_for_service("{}/mavros/set_mode".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/arming".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/takeoff".format(self.id))


        self.setmode_service = rospy.ServiceProxy("{}/mavros/set_mode".format(self.id), SetMode)
        self.arm_service = rospy.ServiceProxy("{}/mavros/cmd/arming".format(self.id), CommandBool)
        self.takeoff_service = rospy.ServiceProxy("{}/mavros/cmd/takeoff".format(self.id), CommandTOL)
        self.land_service = rospy.ServiceProxy("{}/mavros/cmd/land".format(self.id), CommandTOL)
        self.local_setposition_publisher = rospy.Publisher("{}/mavros/setpoint_position/local".format(self.id), PoseStamped, queue_size=1)
        # self.global_setpoint_publisher = rospy.Publisher("{}/mavros/setpoint_position/global".format(self.id), GlobalPositionTarget, queue_size=1)

        # rospy.Subscriber("{}/mavros/global_position/raw/fix".format(self.id), NavSatFix, self.position)
        rospy.Subscriber("{}/mavros/global_position/global".format(self.id), NavSatFix, self.position)
        rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped, self.localposition)
        rospy.Subscriber("{}/co2".format(self.id), String, self.co2Callback)

        self.logPublisher = rospy.Publisher("{}/log".format(self.id), String, queue_size=1)

        rospy.Service("/{}/command/arm".format(self.id), Empty, self.armcommand)
        rospy.Service("/{}/command/takeoff".format(self.id), Empty, self.takeoff)
        rospy.Service("/{}/command/land".format(self.id), Empty, self.land)
        rospy.Service("/{}/command/rtl".format(self.id), Empty, self.rtl)
        rospy.Service("/{}/command/goto".format(self.id), Empty, self.goto)
        rospy.Service("/{}/command/ddsa".format(self.id), DDSA, self.ddsa)
        rospy.Service("/{}/command/lawnmower".format(self.id), Lawnmower, self.lawnmower)
        rospy.Service("/{}/command/navigate".format(self.id), Navigation, self.navigate)
        rospy.Service("/{}/command/cancel".format(self.id), Empty, self.cancel)
        rospy.Service("/{}/command/hello".format(self.id), Empty, self.hello)

        rospy.Service("/{}/build/ddsa".format(self.id), DDSAWaypoints, self.build_ddsa)
        rospy.Service("/{}/build/lawnmower".format(self.id), LawnmowerWaypoints, self.build_lawnmower)

        print "Setup complete"

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Drone command service.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    # parser.add_argument('alt', type=int, help='Altitude to fly.')
    # parser.add_argument('index', type=int, help='Index in swarm.')
    # parser.add_argument('size', type=int, help='Swarm size.')
    args = parser.parse_args()

    command = DragonflyCommand(args.id)

    command.setup()

    command.loop()

    print "Shutdown"

