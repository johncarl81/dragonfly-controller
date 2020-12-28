#! /usr/bin/env python
import rospy, time, argparse, math, std_msgs, pulp

from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State, Waypoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point, TwistStamped
from dragonfly_messages.srv import *
from dragonfly_messages.msg import MissionStep
from waypointUtil import *
from actions.DisarmAction import DisarmAction
from actions.ArmAction import ArmAction
from actions.ActionQueue import ActionQueue
from actions.ModeAction import ModeAction
from actions.SleepAction import SleepAction
from actions.TakeoffAction import TakeoffAction
from actions.ArmedStateAction import ArmedStateAction
from actions.WaypointAction import WaypointAction
from actions.LogAction import LogAction
from actions.StopInPlaceAction import StopInPlaceAction
from actions.WaitForZeroAction import WaitForZeroAction
from actions.FlockingAction import FlockingAction
from actions.LandAction import LandAction
from actions.WaitForDisarmAction import WaitForDisarmAction
from actions.SetPositionAction import SetPositionAction
from actions.MissionStartAction import MissionStartAction
from actions.SemaphoreAction import SemaphoreAction

class MissionStarter:
    start = False

class DragonflyCommand:

    TEST_ALTITUDE = 3

    def __init__(self, id):
        self.zeroing = False
        self.canceled = False
        self.sincezero = 0
        self.id = id
        self.actionqueue = ActionQueue()
        self.mission_starter = MissionStarter()

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

        self.actionqueue.push(ModeAction(self.setmode_service, "STABILIZE")) \
            .push(ArmAction(self.arm_service)) \
            .push(SleepAction(5)) \
            .push(DisarmAction(self.arm_service))

        return EmptyResponse()

    def takeoff(self, operation):
        print "Commanded to takeoff"

        self.actionqueue.push(ArmedStateAction(self.id)) \
            .push(ModeAction(self.setmode_service, "STABILIZE")) \
            .push(ArmAction(self.arm_service)) \
            .push(SleepAction(5)) \
            .push(ModeAction(self.setmode_service, "GUIDED")) \
            .push(TakeoffAction(self.takeoff_service, self.TEST_ALTITUDE))

        return EmptyResponse()

    def land(self, operation):
        print "Commanded to land"

        self.actionqueue.push(LandAction(self.land_service)) \
            .push(WaitForDisarmAction(self.id)) \
            .push(ModeAction(self.setmode_service, "STABILIZE"))

        return EmptyResponse()

    def rtl(self, operation):
        print "Commanded to land"

        self.setmode("RTL")

        self.cancel(None)

        return EmptyResponse()

    def goto(self, operation):
        print "Commanded to goto"

        self.actionqueue.push(SetPositionAction(self.local_setposition_publisher, 0, 10, self.TEST_ALTITUDE)) \
            .push(SleepAction(10)) \
            .push(SetPositionAction(self.local_setposition_publisher, 0, 0, self.TEST_ALTITUDE)) \
            .push(SleepAction(10))

        return EmptyResponse()

    def home(self, operation):
        print "Commanded to home"

        self.actionqueue.push(SetPositionAction(self.local_setposition_publisher, 0, 0, 10))

        return EmptyResponse()

    def build_ddsa_waypoints(self, startingWaypoint, walk, stacks, loops, radius, stepLength, altitude):
        ddsaWaypoints = build3DDDSAWaypoints(Span(walk), stacks, 1, 0, loops, radius, stepLength)

        localWaypoints = []
        for localwaypoint in ddsaWaypoints:
            localWaypoints.append(createWaypoint(startingWaypoint.x + localwaypoint.x, startingWaypoint.y + localwaypoint.y, altitude + localwaypoint.z))

        return localWaypoints


    def build_ddsa(self, operation):
        ddsaWaypoints = self.build_ddsa_waypoints(self.localposition, operation.walk, operation.stacks, operation.loops, operation.radius, operation.stepLength, operation.altitude)

        waypoints = []
        for localwaypoint in ddsaWaypoints:
            waypoints.append(createLatLon(localwaypoint.pose.position, self.localposition, self.position))

        return DDSAWaypointsResponse(waypoints=waypoints)

    def ddsa(self, operation):
        print "Commanded to ddsa"
        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Started")) \
            .push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        waypoints = self.build_ddsa_waypoints(self.localposition, operation.walk, operation.stacks, operation.loops, operation.radius, operation.stepLength, operation.altitude)

        self.runWaypoints(waypoints, operation.waitTime, operation.distanceThreshold)

        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Finished"))

        return DDSAResponse(success=True, message="Commanded {} to DDSA.".format(self.id))

    def build_lawnmower_waypoints(self, walkBoundary, boundary, walk, altitude, stacks, stepLength):
        lawnmowerLocalWaypoints = []

        if walkBoundary:
            wrappedGPSBoundary = []
            wrappedGPSBoundary.extend(boundary)
            wrappedGPSBoundary.append(boundary[0])

            for waypoint in wrappedGPSBoundary:
                lawnmowerLocalWaypoints.append(buildRelativeWaypoint(self.localposition, self.position, waypoint, altitude))

        lawnmowerLocalWaypoints.extend(build3DLawnmowerWaypoints(Span(walk), altitude, self.localposition, self.position, stacks, boundary, stepLength))

        return lawnmowerLocalWaypoints

    def build_lawnmower(self, operation):

        waypoints = []
        for lawnmowerLocalWaypoint in self.build_lawnmower_waypoints(operation.walkBoundary, operation.boundary, operation.walk, operation.altitude, operation.stacks, operation.stepLength):
            waypoints.append(createLatLon(lawnmowerLocalWaypoint.pose.position, self.localposition, self.position))

        return LawnmowerWaypointsResponse(waypoints=waypoints)

    def lawnmower(self, operation):
        print "Commanded to lawnmower"
        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Started")) \
            .push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        print "Position: {} {} {}".format(self.localpositionpose.x, self.localpositionpose.y, self.localpositionpose.z)

        waypoints = self.build_lawnmower_waypoints(operation.walkBoundary, operation.boundary, operation.walk, operation.altitude, operation.stacks, operation.stepLength)

        self.runWaypoints(waypoints, operation.waitTime, operation.distanceThreshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Finished"))

        return LawnmowerResponse(success=True, message="Commanded {} to lawnmower.".format(self.id))

    def navigate(self, operation):
        print "Commanded to navigate"
        self.actionqueue.push(LogAction(self.logPublisher, "Navigation started")) \
            .push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        print "{} {}".format(self.localposition.z, self.position.altitude)

        localWaypoints = []
        for waypoint in operation.waypoints:
            print "{} {} {}".format(self.localposition.z, self.position.altitude, waypoint.relativeAltitude)
            localWaypoints.append(buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relativeAltitude))

        self.runWaypoints(localWaypoints, operation.waitTime, operation.distanceThreshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Navigation Finished"))

        return NavigationResponse(success=True, message="Commanded {} to navigate.".format(self.id))

    def findWaypoint(self, waypoint_name, waypoints):
        for waypoint in waypoints:
            if waypoint.name == waypoint_name:
                return [buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relativeAltitude), waypoint.distanceThreshold]
        return [None, None]

    def findBoundary(self, boundary_name, boundaries):
        for boundary in boundaries:
            if boundary.name == boundary_name:
                return boundary.points
        return None

    def mission(self, operation):
        self.cancel(None)

        for step in operation.steps:
            if step.msg_type == MissionStep.TYPE_START:
                print "Start"
                self.actionqueue.push(MissionStartAction(self.mission_starter))
            elif step.msg_type == MissionStep.TYPE_TAKEOFF:
                print "Takeoff"
                self.actionqueue.push(ArmedStateAction(self.id)) \
                    .push(ModeAction(self.setmode_service, "STABILIZE")) \
                    .push(ArmAction(self.arm_service)) \
                    .push(SleepAction(5)) \
                    .push(ModeAction(self.setmode_service, "GUIDED")) \
                    .push(TakeoffAction(self.takeoff_service, step.takeoff.altitude)) \
                    .push(SleepAction(5))
            elif step.msg_type == MissionStep.TYPE_SLEEP:
                print "Sleep"
                self.actionqueue.push(SleepAction(step.sleep.duration))
            elif step.msg_type == MissionStep.TYPE_LAND:
                print "Land"
                self.actionqueue.push(LandAction(self.land_service)) \
                    .push(WaitForDisarmAction(self.id)) \
                    .push(ModeAction(self.setmode_service, "STABILIZE"))
            elif step.msg_type == MissionStep.TYPE_GOTO_WAYPOINT:
                print "Waypoint"
                [waypoint, distanceThreshold] = self.findWaypoint(step.goto.waypoint, operation.waypoints)
                if waypoint is not None:
                    self.actionqueue.push(WaypointAction(self.id, self.local_setposition_publisher, waypoint, distanceThreshold))
            elif step.msg_type == MissionStep.TYPE_SEMAPHORE:
                print "Semaphore"
                self.actionqueue.push(SemaphoreAction(self.id, step.semaphore.id, step.semaphore.drones))
            elif step.msg_type == MissionStep.TYPE_RTL:
                print "RTL"
                self.actionqueue.push(ModeAction(self.setmode_service, 'RTL'))
            elif step.msg_type == MissionStep.TYPE_DDSA:
                print "DDSA"
                [waypoint, distanceThreshold] = self.findWaypoint(step.ddsa.waypoint, operation.waypoints)
                if waypoint is not None:
                    waypoints = self.build_ddsa_waypoints(waypoint.pose.position, step.ddsa.walk, step.ddsa.stacks, step.ddsa.loops, step.ddsa.radius, step.ddsa.stepLength, step.ddsa.altitude)
                    self.runWaypoints(waypoints, step.ddsa.waitTime, step.ddsa.distanceThreshold)
            elif step.msg_type == MissionStep.TYPE_LAWNMOWER:
                print "Lawnmower"
                boundary = self.findBoundary(step.lawnmower.boundary, operation.boundaries)
                if boundary is not None:
                    waypoints = self.build_lawnmower_waypoints(step.lawnmower.walkBoundary, boundary, step.lawnmower.walk, step.lawnmower.altitude, step.lawnmower.stacks, step.lawnmower.stepLength)
                    self.runWaypoints(waypoints, step.lawnmower.waitTime, step.lawnmower.distanceThreshold)
            elif step.msg_type == MissionStep.TYPE_NAVIGATION:
                print "Navigation"
                localWaypoints = []
                for waypoint in step.navigation.waypoints:
                    print "{} {} {}".format(self.localposition.z, self.position.altitude, waypoint.relativeAltitude)
                    localWaypoints.append(buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relativeAltitude))
                self.runWaypoints(localWaypoints, step.navigation.waitTime, step.navigation.distanceThreshold)


    def start_mission(self, operation):
        self.canceled = False
        self.mission_starter.start = True

    def runWaypoints(self, waypoints, waitTime, distanceThreshold):

        for waypoint in waypoints:
            self.actionqueue.push(WaypointAction(self.id, self.local_setposition_publisher, waypoint, distanceThreshold))
            if waitTime > 0:
                self.actionqueue.push(SleepAction(waitTime))
            self.actionqueue.push(WaitForZeroAction(self))

        return EmptyResponse()

    def flock(self, flockCommand):

        self.actionqueue.push(ModeAction(self.setmode_service, 'GUIDED')) \
            .push(FlockingAction(self.id, self.local_setvelocity_publisher, flockCommand.x, flockCommand.y, flockCommand.leader))

        return FlockResponse(success=True, message="Flocking {} with {}.".format(self.id, flockCommand.leader))

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
        self.zeroing = self.sincezero < 60
        if self.zeroing and not previous:
            self.logPublisher.publish('Zeroing')
        elif not self.zeroing and previous:
            self.logPublisher.publish('Finished zeroing')

    def cancel(self, operation):
        self.canceled = True
        self.actionqueue.stop()
        self.actionqueue.push(StopInPlaceAction(self.id, self.local_setposition_publisher))

        return EmptyResponse()

    def loop(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.actionqueue.step()
            rate.sleep()

    def setup(self):
        rospy.init_node("{}_remote_service".format(self.id), anonymous=True)

        rospy.wait_for_service("{}/mavros/set_mode".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/arming".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/takeoff".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/land".format(self.id))


        self.setmode_service = rospy.ServiceProxy("{}/mavros/set_mode".format(self.id), SetMode)
        self.arm_service = rospy.ServiceProxy("{}/mavros/cmd/arming".format(self.id), CommandBool)
        self.takeoff_service = rospy.ServiceProxy("{}/mavros/cmd/takeoff".format(self.id), CommandTOL)
        self.land_service = rospy.ServiceProxy("{}/mavros/cmd/land".format(self.id), CommandTOL)
        self.local_setposition_publisher = rospy.Publisher("{}/mavros/setpoint_position/local".format(self.id), PoseStamped, queue_size=1)
        self.local_setvelocity_publisher = rospy.Publisher("{}/mavros/setpoint_velocity/cmd_vel".format(self.id), TwistStamped, queue_size=1)
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
        rospy.Service("/{}/command/home".format(self.id), Empty, self.home)
        rospy.Service("/{}/command/goto".format(self.id), Empty, self.goto)
        rospy.Service("/{}/command/ddsa".format(self.id), DDSA, self.ddsa)
        rospy.Service("/{}/command/lawnmower".format(self.id), Lawnmower, self.lawnmower)
        rospy.Service("/{}/command/navigate".format(self.id), Navigation, self.navigate)
        rospy.Service("/{}/command/mission".format(self.id), Mission, self.mission)
        rospy.Service("/{}/command/start_mission".format(self.id), Empty, self.start_mission)
        rospy.Service("/{}/command/flock".format(self.id), Flock, self.flock)
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

