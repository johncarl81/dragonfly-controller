#!/usr/bin/env python
import argparse
import time
import threading
import rospy

from datetime import datetime, timedelta
from dragonfly_messages.msg import MissionStep
from dragonfly_messages.srv import *
from geometry_msgs.msg import TwistStamped
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSet
from mavros_msgs.msg import ParamValue
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from actions import *
from waypointUtil import *
from boundaryUtil import *


class MissionStarter:
    start = False


class DragonflyCommand:
    TEST_ALTITUDE = 3

    def __init__(self, id):
        self.zeroing = False
        self.canceled = False
        self.sincezero = datetime.now()
        self.id = id
        self.actionqueue = ActionQueue()
        self.mission_starter = MissionStarter()
        self.position = None
        self.local_position = None
        self.localposition = None
        self.orientation = None

    def setmode(self, mode):
        print("Set Mode {}".format(mode))
        print(self.setmode_service(custom_mode=mode))
        time.sleep(1)

    def arm(self):
        print("Arming")
        print(self.arm_service(True))

        time.sleep(5)

    def disarm(self):
        print("Disarming")
        print(self.arm_service(False))

    def hello(self, command):
        print("hello @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))
        return SimpleResponse()

    def armcommand(self, operation):
        print("Commanded to arm @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))

        self.actionqueue.push(ModeAction(self.setmode_service, "STABILIZE")) \
            .push(ArmAction(self.logPublisher, self.arm_service)) \
            .push(SleepAction(5)) \
            .push(DisarmAction(self.logPublisher, self.arm_service))

        return SimpleResponse()

    def takeoff(self, operation):
        print("Commanded to takeoff @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))

        self.actionqueue.push(ArmedStateAction(self.logPublisher, self.id)) \
            .push(ModeAction(self.setmode_service, "STABILIZE")) \
            .push(ArmAction(self.logPublisher, self.arm_service)) \
            .push(SleepAction(5)) \
            .push(ModeAction(self.setmode_service, "GUIDED")) \
            .push(TakeoffAction(self.logPublisher, self.takeoff_service, self.TEST_ALTITUDE))

        return SimpleResponse()

    def land(self, operation):
        print("Commanded to land @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))

        self.actionqueue.push(LandAction(self.logPublisher, self.land_service)) \
            .push(WaitForDisarmAction(self.id, self.logPublisher)) \
            .push(ModeAction(self.setmode_service, "STABILIZE"))

        return SimpleResponse()

    def rtl(self, operation):
        print("Commanded to RTL @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))

        self.cancel()

        self.setmode("RTL")
        self.logPublisher.publish("RTL")

        return SimpleResponse()

    def goto(self, operation):
        print("Commanded to goto @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))

        self.actionqueue.push(
            SetPositionAction(self.local_setposition_publisher, 0, 10, self.TEST_ALTITUDE, self.orientation)) \
            .push(SleepAction(10)) \
            .push(SetPositionAction(self.local_setposition_publisher, 0, 0, self.TEST_ALTITUDE, self.orientation)) \
            .push(SleepAction(10))

        return SimpleResponse()

    def home(self, operation):
        print("Commanded to home @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))

        self.actionqueue.push(SetPositionAction(self.local_setposition_publisher, 0, 0, 10, self.orientation))

        return SimpleResponse()

    def build_ddsa_waypoints(self, startingWaypoint, walk, stacks, swarm_size, swarm_index, loops, radius, stepLength, altitude, orientation):
        ddsaWaypoints = build3DDDSAWaypoints(Span(walk), stacks, swarm_size, swarm_index, loops, radius, stepLength)

        localWaypoints = []
        for localwaypoint in ddsaWaypoints:
            localWaypoints.append(createWaypoint(
                startingWaypoint.x + localwaypoint.x,
                startingWaypoint.y + localwaypoint.y,
                altitude + localwaypoint.z,
                orientation
            ))

        return localWaypoints

    def build_ddsa(self, operation):
        ddsaWaypoints = self.build_ddsa_waypoints(self.localposition, operation.walk, operation.stacks, 1, 0, operation.loops,
                                                  operation.radius, operation.stepLength, operation.altitude,
                                                  self.orientation)

        waypoints = []
        for localwaypoint in ddsaWaypoints:
            waypoints.append(createLatLon(localwaypoint.pose.position, self.localposition, self.position))

        return DDSAWaypointsResponse(waypoints=waypoints)

    def ddsa(self, operation):
        print("Commanded to ddsa @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))
        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Started")) \
            .push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        waypoints = self.build_ddsa_waypoints(self.localposition, operation.walk, operation.stacks, 1, 0, operation.loops,
                                              operation.radius, operation.stepLength, operation.altitude,
                                              self.orientation)

        self.runWaypoints("DDSA", waypoints, operation.waitTime, operation.distanceThreshold)

        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Finished"))

        return DDSAResponse(success=True, message="Commanded {} to DDSA.".format(self.id))

    def build_lawnmower_waypoints(self, walkBoundary, boundary, walk, altitude, stacks, stepLength, orientation):
        lawnmowerLocalWaypoints = []

        if walkBoundary:
            wrappedGPSBoundary = []
            wrappedGPSBoundary.extend(boundary)
            wrappedGPSBoundary.append(boundary[0])

            for waypoint in wrappedGPSBoundary:
                lawnmowerLocalWaypoints.append(
                    buildRelativeWaypoint(self.localposition, self.position, waypoint, altitude, self.orientation))

        lawnmowerLocalWaypoints.extend(
            build3DLawnmowerWaypoints(Span(walk), altitude, self.localposition, self.position, stacks, boundary,
                                      stepLength, orientation))

        return lawnmowerLocalWaypoints

    def build_lawnmower(self, operation):

        waypoints = []
        for lawnmowerLocalWaypoint in self.build_lawnmower_waypoints(operation.walkBoundary, operation.boundary,
                                                                     operation.walk, operation.altitude,
                                                                     operation.stacks, operation.stepLength,
                                                                     self.orientation):
            waypoints.append(createLatLon(lawnmowerLocalWaypoint.pose.position, self.localposition, self.position))

        return LawnmowerWaypointsResponse(waypoints=waypoints)

    def lawnmower(self, operation):
        print("Commanded to lawnmower")
        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Started")) \
            .push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        print("Position: {} {} {}".format(self.localposition.x, self.localposition.y, self.localposition.z))

        waypoints = self.build_lawnmower_waypoints(operation.walkBoundary, operation.boundary, operation.walk,
                                                   operation.altitude, operation.stacks, operation.stepLength,
                                                   self.orientation)

        self.runWaypoints("Lawnmower", waypoints, operation.waitTime, operation.distanceThreshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Finished"))

        return LawnmowerResponse(success=True, message="Commanded {} to lawnmower.".format(self.id))

    def navigate(self, operation):
        print("Commanded to navigate")
        self.actionqueue.push(LogAction(self.logPublisher, "Navigation started")) \
            .push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        print("{} {}".format(self.localposition.z, self.position.altitude))

        localWaypoints = []
        for waypoint in operation.waypoints:
            print("{} {} {}".format(self.localposition.z, self.position.altitude, waypoint.relativeAltitude))
            localWaypoints.append(
                buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relativeAltitude,
                                      self.orientation))

        self.runWaypoints("Navigation", localWaypoints, operation.waitTime, operation.distanceThreshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Navigation Finished"))

        return NavigationResponse(success=True, message="Commanded {} to navigate.".format(self.id))

    def findWaypoint(self, waypoint_name, waypoints):
        for waypoint in waypoints:
            if waypoint.name == waypoint_name:
                return [buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relativeAltitude,
                                              self.orientation), waypoint.distanceThreshold]
        return [None, None]

    def findBoundary(self, boundary_name, boundaries):
        for boundary in boundaries:
            if boundary.name == boundary_name:
                return boundary.points
        return None

    def setup_drone(self, operation):
        print("Setup @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))

        param_value = ParamValue()
        param_value.integer = operation.rtl_altitude
        result = self.setparam_service(param_id = 'RTL_ALT', value=param_value)

        self.rtl_boundary = operation.rtl_boundary
        self.max_altitude = operation.max_altitude

        self.logPublisher.publish("Setup Success: {}".format(result.success))

        return SetupResponse(success=result.success, message="Setup {}".format(self.id))

    def rtl_boundary_check(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.localposition is not None and self.position is not None and not self.canceled:
                if self.localposition.z > self.max_altitude:
                    self.logPublisher.publish("Exceeded maximum altitude of {}m".format(self.max_altitude))
                    self.rtl(None)
                if self.rtl_boundary is not None and not isInside(self.position, self.rtl_boundary.points):
                    self.logPublisher.publish("Exceeded RTL Boundary at {}, {}".format(self.position.longitude, self.position.latitude))
                    self.rtl(None)

            rate.sleep()

    def mission(self, operation):
        self.cancel()

        for step in operation.steps:
            if step.msg_type == MissionStep.TYPE_START:
                print("Start")
                self.actionqueue.push(MissionStartAction(self.logPublisher, self.mission_starter))
            elif step.msg_type == MissionStep.TYPE_TAKEOFF:
                print("Takeoff")
                self.actionqueue.push(ArmedStateAction(self.logPublisher, self.id)) \
                    .push(ModeAction(self.setmode_service, "STABILIZE")) \
                    .push(ArmAction(self.logPublisher, self.arm_service)) \
                    .push(SleepAction(5)) \
                    .push(ModeAction(self.setmode_service, "GUIDED")) \
                    .push(TakeoffAction(self.logPublisher, self.takeoff_service, step.takeoff.altitude)) \
                    .push(SleepAction(5))
            elif step.msg_type == MissionStep.TYPE_SLEEP:
                print("Sleep")
                self.actionqueue.push(SleepAction(step.sleep.duration))
            elif step.msg_type == MissionStep.TYPE_LAND:
                print("Land")
                self.actionqueue.push(LandAction(self.logPublisher, self.land_service)) \
                    .push(WaitForDisarmAction(self.id, self.logPublisher)) \
                    .push(ModeAction(self.setmode_service, "STABILIZE"))
            elif step.msg_type == MissionStep.TYPE_GOTO_WAYPOINT:
                print("Waypoint")
                [waypoint, distanceThreshold] = self.findWaypoint(step.goto.waypoint, operation.waypoints)
                if waypoint is not None:
                    self.actionqueue.push(LogAction(self.logPublisher, "Goto {}".format(step.goto.waypoint))) \
                        .push(WaypointAction(self.id, self.local_setposition_publisher, waypoint, distanceThreshold))
            elif step.msg_type == MissionStep.TYPE_SEMAPHORE:
                print("Semaphore")
                self.actionqueue.push(LogAction(self.logPublisher, "Waiting for semaphore...")) \
                    .push(SemaphoreAction(self.id, step.semaphore.id, step.semaphore.drones)) \
                    .push(LogAction(self.logPublisher, "Semaphore reached"))
            elif step.msg_type == MissionStep.TYPE_RTL:
                print("RTL")
                self.actionqueue.push(LogAction(self.logPublisher, "RTL".format(step.goto.waypoint))) \
                    .push(ModeAction(self.setmode_service, 'RTL')) \
                    .push(WaitForDisarmAction(self.id, self.logPublisher))
            elif step.msg_type == MissionStep.TYPE_DDSA:
                print("DDSA")
                self.actionqueue.push(LogAction(self.logPublisher, "DDSA"))
                [waypoint, distanceThreshold] = self.findWaypoint(step.ddsa.waypoint, operation.waypoints)
                if waypoint is not None:
                    waypoints = self.build_ddsa_waypoints(waypoint.pose.position, step.ddsa.walk, step.ddsa.stacks,
                                                          step.ddsa.swarm_size, step.ddsa.swarm_index,
                                                          step.ddsa.loops, step.ddsa.radius, step.ddsa.stepLength,
                                                          step.ddsa.altitude, self.orientation)
                    self.actionqueue.push(AltitudeAction(self.id,
                                                         self.local_setposition_publisher,
                                                         waypoint.pose.position.z + (step.ddsa.radius * step.ddsa.swarm_index),
                                                         step.ddsa.distanceThreshold))
                    position_waypoint = createWaypoint(waypoint.pose.position.x - (step.ddsa.radius * step.ddsa.swarm_index),
                                                       waypoint.pose.position.y,
                                                       waypoint.pose.position.z + (step.ddsa.radius * step.ddsa.swarm_index),
                                                       self.orientation)
                    self.actionqueue.push(WaypointAction(self.id, self.local_setposition_publisher, position_waypoint, step.ddsa.distanceThreshold))
                    self.runWaypoints("DDSA", waypoints, step.ddsa.waitTime, step.ddsa.distanceThreshold)
            elif step.msg_type == MissionStep.TYPE_LAWNMOWER:
                print("Lawnmower")
                self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower"))
                boundary = self.findBoundary(step.lawnmower.boundary, operation.boundaries)
                if boundary is not None:
                    waypoints = self.build_lawnmower_waypoints(step.lawnmower.walkBoundary, boundary,
                                                               step.lawnmower.walk, step.lawnmower.altitude,
                                                               step.lawnmower.stacks, step.lawnmower.stepLength,
                                                               self.orientation)
                    self.runWaypoints("Lawnmower", waypoints, step.lawnmower.waitTime, step.lawnmower.distanceThreshold)
            elif step.msg_type == MissionStep.TYPE_NAVIGATION:
                print("Navigation")
                self.actionqueue.push(LogAction(self.logPublisher, "Navigation"))
                localWaypoints = []
                for waypoint in step.navigation.waypoints:
                    localWaypoints.append(
                        buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relativeAltitude,
                                              self.orientation))
                self.runWaypoints("Navigation", localWaypoints, step.navigation.waitTime,
                                  step.navigation.distanceThreshold)
            elif step.msg_type == MissionStep.TYPE_FLOCK:
                print("Flock")
                self.actionqueue.push(LogAction(self.logPublisher, "Flock")) \
                    .push(FlockingAction(self.id, self.logPublisher, self.local_setvelocity_publisher, step.flock.x,
                                         step.flock.y, step.flock.leader))
            elif step.msg_type == MissionStep.TYPE_GRADIENT:
                print("Gradient")
                self.actionqueue.push(LogAction(self.logPublisher, "Following Gradient")) \
                    .push(
                    GradientAction(self.id, self.logPublisher, self.local_setvelocity_publisher, step.gradient.drones))

        self.actionqueue.push(LogAction(self.logPublisher, "Mission complete"))
        self.logPublisher.publish("Mission with {} steps setup".format(len(operation.steps)))

        return MissionResponse(success=True, message="{} mission received.".format(self.id))

    def start_mission(self, operation):
        print("Commanded to navigate @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))

        self.canceled = False
        self.mission_starter.start = True

        return SimpleResponse()

    def runWaypoints(self, waypoints_name, waypoints, waitTime, distanceThreshold):

        for i, waypoint in enumerate(waypoints):
            self.actionqueue.push(
                LogAction(self.logPublisher, "Goto {} {}/{}".format(waypoints_name, i + 1, len(waypoints))))
            self.actionqueue.push(
                WaypointAction(self.id, self.local_setposition_publisher, waypoint, distanceThreshold))
            if waitTime > 0:
                self.actionqueue.push(SleepAction(waitTime))
            self.actionqueue.push(WaitForZeroAction(self.logPublisher, self))

        return SimpleResponse()

    def flock(self, flockCommand):

        self.actionqueue.push(ModeAction(self.setmode_service, 'GUIDED')) \
            .push(
            FlockingAction(self.id, self.logPublisher, self.local_setvelocity_publisher, flockCommand.x, flockCommand.y,
                           flockCommand.leader))

        return FlockResponse(success=True, message="Flocking {} with {}.".format(self.id, flockCommand.leader))

    def position_callbak(self, data):
        # print data
        self.position = data

    def localposition_callback(self, data):
        self.localposition = data.pose.position
        self.orientation = data.pose.orientation

    def co2Callback(self, data):
        if data.data.startswith('W') or data.data.startswith('Z'):
            self.sincezero = datetime.now()
        previous = self.zeroing
        self.zeroing = datetime.now() - self.sincezero < timedelta(seconds=10)
        if self.zeroing and not previous:
            self.logPublisher.publish('Zeroing')
        elif not self.zeroing and previous:
            self.logPublisher.publish('Finished zeroing')

    def cancelCommand(self, operation):
        print("Commanded to cancel @ {}".format(datetime.fromtimestamp(operation.command_time.secs)))
        
        self.cancel()

        return SimpleResponse()

    def cancel(self):
        self.canceled = True
        self.actionqueue.stop()
        self.actionqueue.push(StopInPlaceAction(self.id, self.logPublisher, self.local_setposition_publisher))

    def loop(self):
        try:
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                self.actionqueue.step()
                rate.sleep()
        except ROSInterruptException:
            print("Shutting down...")

    def setup(self):
        rospy.init_node("{}_remote_service".format(self.id), anonymous=True)

        self.rtl_boundary = None
        self.max_altitude = 100

        rospy.wait_for_service("{}/mavros/set_mode".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/arming".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/takeoff".format(self.id))
        rospy.wait_for_service("{}/mavros/cmd/land".format(self.id))

        self.setparam_service = rospy.ServiceProxy("{}/mavros/param/set".format(self.id), ParamSet)
        self.setmode_service = rospy.ServiceProxy("{}/mavros/set_mode".format(self.id), SetMode)
        self.arm_service = rospy.ServiceProxy("{}/mavros/cmd/arming".format(self.id), CommandBool)
        self.takeoff_service = rospy.ServiceProxy("{}/mavros/cmd/takeoff".format(self.id), CommandTOL)
        self.land_service = rospy.ServiceProxy("{}/mavros/cmd/land".format(self.id), CommandTOL)
        self.local_setposition_publisher = rospy.Publisher("{}/mavros/setpoint_position/local".format(self.id),
                                                           PoseStamped, queue_size=1)
        self.local_setvelocity_publisher = rospy.Publisher("{}/mavros/setpoint_velocity/cmd_vel".format(self.id),
                                                           TwistStamped, queue_size=1)
        # self.global_setpoint_publisher = rospy.Publisher("{}/mavros/setpoint_position/global".format(self.id), GlobalPositionTarget, queue_size=1)

        # rospy.Subscriber("{}/mavros/global_position/raw/fix".format(self.id), NavSatFix, self.position_callbak)
        rospy.Subscriber("{}/mavros/global_position/global".format(self.id), NavSatFix, self.position_callbak)
        rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped, self.localposition_callback)
        rospy.Subscriber("{}/co2".format(self.id), String, self.co2Callback)

        self.logPublisher = rospy.Publisher("{}/log".format(self.id), String, queue_size=1)

        rospy.Service("/{}/command/arm".format(self.id), Simple, self.armcommand)
        rospy.Service("/{}/command/takeoff".format(self.id), Simple, self.takeoff)
        rospy.Service("/{}/command/land".format(self.id), Simple, self.land)
        rospy.Service("/{}/command/rtl".format(self.id), Simple, self.rtl)
        rospy.Service("/{}/command/home".format(self.id), Simple, self.home)
        rospy.Service("/{}/command/goto".format(self.id), Simple, self.goto)
        rospy.Service("/{}/command/ddsa".format(self.id), DDSA, self.ddsa)
        rospy.Service("/{}/command/lawnmower".format(self.id), Lawnmower, self.lawnmower)
        rospy.Service("/{}/command/navigate".format(self.id), Navigation, self.navigate)
        rospy.Service("/{}/command/mission".format(self.id), Mission, self.mission)
        rospy.Service("/{}/command/setup".format(self.id), Setup, self.setup_drone)
        rospy.Service("/{}/command/start_mission".format(self.id), Simple, self.start_mission)
        rospy.Service("/{}/command/flock".format(self.id), Flock, self.flock)
        rospy.Service("/{}/command/cancel".format(self.id), Simple, self.cancelCommand)
        rospy.Service("/{}/command/hello".format(self.id), Simple, self.hello)

        rospy.Service("/{}/build/ddsa".format(self.id), DDSAWaypoints, self.build_ddsa)
        rospy.Service("/{}/build/lawnmower".format(self.id), LawnmowerWaypoints, self.build_lawnmower)

        print("Setup complete")

        self.boundary_check_thread = threading.Thread(target=self.rtl_boundary_check)
        self.boundary_check_thread.start()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Drone command service.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    # parser.add_argument('alt', type=int, help='Altitude to fly.')
    # parser.add_argument('index', type=int, help='Index in swarm.')
    # parser.add_argument('size', type=int, help='Swarm size.')
    args = parser.parse_args()

    command = DragonflyCommand(args.id)

    command.setup()

    command.loop()
