#!/usr/bin/env python
import argparse
import sys
import threading
import time
from datetime import datetime, timedelta

import rclpy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL, ParamSetV2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from dragonfly_messages.msg import MissionStep
from dragonfly_messages.srv import *
from .actions import *
from .boundaryUtil import *
from .waypointUtil import *


class MissionStarter:
    start = False


class DragonflyCommand:
    TEST_ALTITUDE = 3

    def __init__(self, id, node):
        self.boundary_check_thread = None
        self.node = node
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
        print(self.setmode_service.call(SetMode.Request(custom_mode=mode)))
        time.sleep(1)

    def arm(self):
        print("Arming")
        print(self.arm_service.call(CommandBool.Request(value=True)))

        time.sleep(5)

    def disarm(self):
        print("Disarming")
        print(self.arm_service.call(CommandBool.Request(value=False)))

    def hello(self, request, response):
        print("hello")
        return response

    def armcommand(self, request, response):
        print("Commanded to arm")

        self.actionqueue.push(ModeAction(self.setmode_service, "STABILIZE")) \
            .push(ArmAction(self.logPublisher, self.arm_service)) \
            .push(SleepAction(5)) \
            .push(DisarmAction(self.logPublisher, self.arm_service))

        return response

    def takeoff(self, request, response):
        print("Commanded to takeoff")

        self.actionqueue.push(ArmedStateAction(self.logPublisher, self.id, self.node)) \
            .push(ModeAction(self.setmode_service, "STABILIZE")) \
            .push(ArmAction(self.logPublisher, self.arm_service)) \
            .push(SleepAction(5)) \
            .push(ModeAction(self.setmode_service, "GUIDED")) \
            .push(TakeoffAction(self.logPublisher, self.takeoff_service, self.TEST_ALTITUDE))

        return response

    def land(self, request, response):
        print("Commanded to land")

        self.actionqueue.push(LandAction(self.logPublisher, self.land_service)) \
            .push(WaitForDisarmAction(self.id, self.logPublisher)) \
            .push(ModeAction(self.setmode_service, "STABILIZE"))

        return response

    def rtl_command(self, request, response):
        print("Commanded to RTL")

        self.rtl()

        return response

    def rtl(self):
        self.cancel()

        self.setmode("RTL")
        self.logPublisher.publish(String(data="RTL"))

    def goto(self, request, response):
        print("Commanded to goto")

        self.actionqueue.push(
            SetPositionAction(self.local_setposition_publisher, 0, 10, self.TEST_ALTITUDE, self.orientation)) \
            .push(SleepAction(10)) \
            .push(SetPositionAction(self.local_setposition_publisher, 0, 0, self.TEST_ALTITUDE, self.orientation)) \
            .push(SleepAction(10))

        return response

    def home(self, request, response):
        print("Commanded to home")

        self.actionqueue.push(SetPositionAction(self.local_setposition_publisher, 0, 0, 10, self.orientation))

        return response

    def build_ddsa_waypoints(self, startingWaypoint, walk, stacks, swarm_size, swarm_index, loops, radius, step_length,
                             altitude, orientation):
        ddsaWaypoints = build3DDDSAWaypoints(Span(walk), stacks, swarm_size, swarm_index, loops, radius, step_length)

        localWaypoints = []
        for localwaypoint in ddsaWaypoints:
            localWaypoints.append(createWaypoint(
                startingWaypoint.x + localwaypoint.x,
                startingWaypoint.y + localwaypoint.y,
                altitude + localwaypoint.z,
                orientation
            ))

        return localWaypoints

    def build_ddsa(self, request, response):
        ddsaWaypoints = self.build_ddsa_waypoints(self.localposition, request.walk, request.stacks, 1, 0,
                                                  request.loops,
                                                  request.radius, request.step_length, request.altitude,
                                                  self.orientation)

        waypoints = []
        for localwaypoint in ddsaWaypoints:
            waypoints.append(createLatLon(localwaypoint.pose.position, self.localposition, self.position))

        return DDSAWaypoints.Response(waypoints=waypoints)

    def ddsa(self, request, response):
        print("Commanded to ddsa")
        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Started")) \
            .push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        waypoints = self.build_ddsa_waypoints(self.localposition, request.walk, request.stacks, 1, 0,
                                              request.loops,
                                              request.radius, request.step_length, request.altitude,
                                              self.orientation)

        self.runWaypoints("DDSA", waypoints, request.wait_time, request.distance_threshold)

        self.actionqueue.push(LogAction(self.logPublisher, "DDSA Finished"))

        return DDSA.Response(success=True, message="Commanded {} to DDSA.".format(self.id))

    def build_lawnmower_waypoints(self, walk_boundary, boundary, walk, altitude, stacks, step_length, orientation):
        lawnmowerLocalWaypoints = []

        if walk_boundary:
            wrappedGPSBoundary = []
            wrappedGPSBoundary.extend(boundary)
            wrappedGPSBoundary.append(boundary[0])

            for waypoint in wrappedGPSBoundary:
                lawnmowerLocalWaypoints.append(
                    buildRelativeWaypoint(self.localposition, self.position, waypoint, altitude, self.orientation))

        lawnmowerLocalWaypoints.extend(
            build3DLawnmowerWaypoints(Span(walk), altitude, self.localposition, self.position, stacks, boundary,
                                      step_length, orientation))

        return lawnmowerLocalWaypoints

    def build_lawnmower(self, request, response):

        waypoints = []
        for lawnmowerLocalWaypoint in self.build_lawnmower_waypoints(request.walk_boundary, request.boundary,
                                                                     request.walk, request.altitude,
                                                                     request.stacks, request.step_length,
                                                                     self.orientation):
            waypoints.append(createLatLon(lawnmowerLocalWaypoint.pose.position, self.localposition, self.position))

        return LawnmowerWaypoints.Response(waypoints=waypoints)

    def lawnmower(self, request, response):
        print("Commanded to lawnmower")
        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Started")) \
            .push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        print("Position: {} {} {}".format(self.localposition.x, self.localposition.y, self.localposition.z))

        waypoints = self.build_lawnmower_waypoints(request.walk_boundary, request.boundary, request.walk,
                                                   request.altitude, request.stacks, request.step_length,
                                                   self.orientation)

        self.runWaypoints("Lawnmower", waypoints, request.wait_time, request.distance_threshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower Finished"))
        return Lawnmower.Response(success=True, message="Commanded {} to lawnmower.".format(self.id))

    def navigate(self, request, response):
        print("Commanded to navigate")
        self.actionqueue.push(LogAction(self.logPublisher, "Navigation started")) \
            .push(ModeAction(self.setmode_service, 'GUIDED'))

        self.canceled = False

        print("{} {}".format(self.localposition.z, self.position.altitude))

        localWaypoints = []
        for waypoint in request.waypoints:
            print("{} {} {}".format(self.localposition.z, self.position.altitude, waypoint.relative_altitude))
            localWaypoints.append(
                buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relative_altitude,
                                      self.orientation))

        self.runWaypoints("Navigation", localWaypoints, request.wait_time, request.distance_threshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Navigation Finished"))

        return Navigation.Response(success=True, message="Commanded {} to navigate.".format(self.id))

    def build_curtain_waypoints(self, startWaypoint, endWaypoint, altitude, stacks, orientation):
        waypoints = []

        reverse = False
        for stack in range(stacks):
            if reverse:
                waypoints.append(createWaypoint(endWaypoint.x, endWaypoint.y, altitude + stack, orientation))
                waypoints.append(createWaypoint(startWaypoint.x, startWaypoint.y, altitude + stack, orientation))
            else:
                waypoints.append(createWaypoint(startWaypoint.x, startWaypoint.y, altitude + stack, orientation))
                waypoints.append(createWaypoint(endWaypoint.x, endWaypoint.y, altitude + stack, orientation))
            reverse = not reverse

        return waypoints

    def findWaypoint(self, waypoint_name, waypoints):
        for waypoint in waypoints:
            if waypoint.name == waypoint_name:
                return [buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relative_altitude,
                                              self.orientation), waypoint.distance_threshold]
        return [None, None]

    def findBoundary(self, boundary_name, boundaries):
        for boundary in boundaries:
            if boundary.name == boundary_name:
                return boundary.points
        return None

    def setup_drone(self, request, response):
        print("Setup")

        param_value = ParamValue()
        param_value.integer = request.rtl_altitude
        result = self.setparam_service.call(param_id='RTL_ALT', value=param_value)

        self.rtl_boundary = request.rtl_boundary
        self.max_altitude = request.max_altitude

        self.logPublisher.publish(String(data="Setup Success: {}".format(result.success)))

        return Setup.Response(success=result.success, message="Setup {}".format(self.id))

    def rtl_boundary_check(self):
        rate = self.node.create_rate(10)
        while rclpy.ok():
            if self.localposition is not None and self.position is not None and not self.canceled:
                if self.localposition.z > self.max_altitude:
                    self.logPublisher.publish(String(data="Exceeded maximum altitude of {}m".format(self.max_altitude)))
                    self.rtl()
                if self.rtl_boundary is not None and not isInside(self.position, self.rtl_boundary.points):
                    self.logPublisher.publish(String(data=
                        "Exceeded RTL Boundary at {}, {}".format(self.position.longitude, self.position.latitude)))
                    self.rtl()

            rate.sleep()

    def mission(self, request, response):
        self.cancel()

        for step in request.steps:
            if step.msg_type == MissionStep.TYPE_START:
                print("Start")
                self.actionqueue.push(MissionStartAction(self.logPublisher, self.mission_starter))
            elif step.msg_type == MissionStep.TYPE_TAKEOFF:
                print("Takeoff")
                self.actionqueue.push(ArmedStateAction(self.logPublisher, self.id, self.node)) \
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
                [waypoint, distance_threshold] = self.findWaypoint(step.goto.waypoint, request.waypoints)
                if waypoint is not None:
                    self.actionqueue.push(LogAction(self.logPublisher, "Goto {}".format(step.goto.waypoint))) \
                        .push(WaypointAction(self.id, self.logPublisher, self.local_setposition_publisher, waypoint,
                                             distance_threshold, self.node))
            elif step.msg_type == MissionStep.TYPE_SEMAPHORE:
                print("Semaphore")
                self.actionqueue.push(LogAction(self.logPublisher, "Waiting for semaphore...")) \
                    .push(SemaphoreAction(self.id, step.semaphore.id, step.semaphore.drones, self.node)) \
                    .push(LogAction(self.logPublisher, "Semaphore reached"))
            elif step.msg_type == MissionStep.TYPE_RTL:
                print("RTL")
                self.actionqueue.push(LogAction(self.logPublisher, "RTL".format(step.goto.waypoint))) \
                    .push(ModeAction(self.setmode_service, 'RTL')) \
                    .push(WaitForDisarmAction(self.id, self.logPublisher))
            elif step.msg_type == MissionStep.TYPE_DDSA:
                print("DDSA")
                self.actionqueue.push(LogAction(self.logPublisher, "DDSA"))
                [waypoint, distance_threshold] = self.findWaypoint(step.ddsa.waypoint, request.waypoints)
                if waypoint is not None:
                    waypoints = self.build_ddsa_waypoints(waypoint.pose.position, step.ddsa.walk, step.ddsa.stacks,
                                                          step.ddsa.swarm_size, step.ddsa.swarm_index,
                                                          step.ddsa.loops, step.ddsa.radius, step.ddsa.step_length,
                                                          step.ddsa.altitude, self.orientation)
                    self.actionqueue.push(AltitudeAction(self.id,
                                                         self.local_setposition_publisher,
                                                         waypoint.pose.position.z + (
                                                                 step.ddsa.radius * step.ddsa.swarm_index),
                                                         step.ddsa.distance_threshold,
                                                         self.node))
                    position_waypoint = createWaypoint(
                        waypoint.pose.position.x - (step.ddsa.radius * step.ddsa.swarm_index),
                        waypoint.pose.position.y,
                        waypoint.pose.position.z + (step.ddsa.radius * step.ddsa.swarm_index),
                        self.orientation)
                    self.actionqueue.push(
                        WaypointAction(self.id, self.logPublisher, self.local_setposition_publisher, position_waypoint,
                                       step.ddsa.distance_threshold, self.node))
                    self.runWaypoints("DDSA", waypoints, step.ddsa.wait_time, step.ddsa.distance_threshold)
            elif step.msg_type == MissionStep.TYPE_LAWNMOWER:
                print("Lawnmower")
                self.actionqueue.push(LogAction(self.logPublisher, "Lawnmower"))
                boundary = self.findBoundary(step.lawnmower.boundary, request.boundaries)
                if boundary is not None:
                    waypoints = self.build_lawnmower_waypoints(step.lawnmower.walk_boundary, boundary,
                                                               step.lawnmower.walk, step.lawnmower.altitude,
                                                               step.lawnmower.stacks, step.lawnmower.step_length,
                                                               self.orientation)
                    self.runWaypoints("Lawnmower", waypoints, step.lawnmower.wait_time,
                                      step.lawnmower.distance_threshold)
            elif step.msg_type == MissionStep.TYPE_NAVIGATION:
                print("Navigation")
                self.actionqueue.push(LogAction(self.logPublisher, "Navigation"))
                localWaypoints = []
                for waypoint in step.navigation.waypoints:
                    localWaypoints.append(
                        buildRelativeWaypoint(self.localposition, self.position, waypoint, waypoint.relative_altitude,
                                              self.orientation))
                self.runWaypoints("Navigation", localWaypoints, step.navigation.wait_time,
                                  step.navigation.distance_threshold)
            elif step.msg_type == MissionStep.TYPE_FLOCK:
                print("Flock")
                self.actionqueue.push(LogAction(self.logPublisher, "Flock")) \
                    .push(FlockingAction(self.id, self.logPublisher, self.local_setvelocity_publisher, step.flock.x,
                                         step.flock.y, step.flock.leader, self.node))
            elif step.msg_type == MissionStep.TYPE_GRADIENT:
                print("Gradient")
                self.actionqueue.push(LogAction(self.logPublisher, "Following Gradient")) \
                    .push(
                    GradientAction(self.id, self.logPublisher, self.local_setvelocity_publisher, step.gradient.drones,
                                   self.node))
            elif step.msg_type == MissionStep.TYPE_CURTAIN:
                print("Curtain")
                self.actionqueue.push(LogAction(self.logPublisher, "Curtain"))
                [startWaypoint, distance_threshold] = self.findWaypoint(step.curtain.start_waypoint,
                                                                        request.waypoints)
                [endWaypoint, distance_threshold] = self.findWaypoint(step.curtain.end_waypoint, request.waypoints)
                if startWaypoint is not None and endWaypoint is not None:
                    localWaypoints = self.build_curtain_waypoints(startWaypoint.pose.position,
                                                                  endWaypoint.pose.position,
                                                                  step.curtain.altitude,
                                                                  step.curtain.stacks,
                                                                  self.orientation)
                    self.runWaypoints("Curtain", localWaypoints, 0, step.curtain.distance_threshold)

        self.actionqueue.push(LogAction(self.logPublisher, "Mission complete"))
        self.logPublisher.publish(String(data="Mission with {} steps setup".format(len(request.steps))))

        return Mission.Response(success=True, message="{} mission received.".format(self.id))

    def start_mission(self, request, response):
        print("Commanded to navigate")

        self.canceled = False
        self.mission_starter.start = True

        return

    def runWaypoints(self, waypoints_name, waypoints, wait_time, distance_threshold):

        for i, waypoint in enumerate(waypoints):
            self.actionqueue.push(
                LogAction(self.logPublisher, "Goto {} {}/{}".format(waypoints_name, i + 1, len(waypoints))))
            self.actionqueue.push(
                WaypointAction(self.id, self.logPublisher, self.local_setposition_publisher, waypoint,
                               distance_threshold, self.node))
            if wait_time > 0:
                self.actionqueue.push(SleepAction(wait_time))
            self.actionqueue.push(WaitForZeroAction(self.logPublisher, self))

        return

    def flock(self, request, response):
        flockCommand = request.steps[0]  # @TODO: check if this is right
        self.actionqueue.push(ModeAction(self.setmode_service, 'GUIDED')) \
            .push(
            FlockingAction(self.id, self.logPublisher, self.local_setvelocity_publisher, flockCommand.x, flockCommand.y,
                           flockCommand.leader, self.node))

        return Flock.Response(success=True, message="Flocking {} with {}.".format(self.id, flockCommand.leader))

    def position_callback(self, data):
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
            self.logPublisher.publish(String(data='Zeroing'))
        elif not self.zeroing and previous:
            self.logPublisher.publish(String(data='Finished zeroing'))

    def cancelCommand(self, request, response):
        print("Commanded to cancel")

        self.cancel()

        return response

    def cancel(self):
        self.canceled = True
        self.actionqueue.stop()
        self.actionqueue.push(
            StopInPlaceAction(self.id, self.logPublisher, self.local_setposition_publisher, self.node))

    def loop(self):
        try:
            rate = self.node.create_rate(1)
            while rclpy.ok():
                self.actionqueue.step()
                rate.sleep()
        except KeyboardInterrupt:
            print("Shutting down...")

    def create_client_and_wait(self, type, name):
        client = self.node.create_client(type, name)
        # while not client.wait_for_service(timeout_sec=1.0):
        #    self.node.get_logger().info("{} service not available, waiting again...".format(name))
        return client

    def create_command(self, name, callback, type=Simple):
        self.node.create_service(type, "/{}/command/{}".format(self.id, name), callback)

    def setup(self):
        self.rtl_boundary = None
        self.max_altitude = 100

        self.setparam_service = self.create_client_and_wait(ParamSetV2, "/{}/mavros/param/set".format(self.id))
        self.setmode_service = self.create_client_and_wait(SetMode, "/{}/mavros/set_mode".format(self.id))
        self.arm_service = self.create_client_and_wait(CommandBool, "/{}/mavros/cmd/arming".format(self.id))
        self.takeoff_service = self.create_client_and_wait(CommandTOL, "/{}/mavros/cmd/takeoff".format(self.id))
        self.land_service = self.create_client_and_wait(CommandTOL, "/{}/mavros/cmd/land".format(self.id))
        self.local_setposition_publisher = self.node.create_publisher(PoseStamped,
                                                                      "/{}/mavros/setpoint_position/local".format(
                                                                          self.id), qos_profile=QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=10))
        self.local_setvelocity_publisher = self.node.create_publisher(TwistStamped,
                                                                      "/{}/mavros/setpoint_velocity/cmd_vel".format(
                                                                          self.id), qos_profile=QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=10))
        # self.global_setpoint_publisher = self.node.create_publisher(GlobalPositionTarget, "/{}/mavros/setpoint_position/global".format(self.id), 10)

        # self.node.create_subscription(NavSatFix, "/{}/mavros/global_position/raw/fix".format(self.id), self.position_callback, 10)
        self.node.create_subscription(NavSatFix, "/{}/mavros/global_position/global".format(self.id),
                                      self.position_callback,
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(PoseStamped, "/{}/mavros/local_position/pose".format(self.id),
                                      self.localposition_callback,
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.node.create_subscription(String, "/{}/co2".format(self.id), self.co2Callback,
                                      qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

        self.logPublisher = self.node.create_publisher(String, "{}/log".format(self.id), qos_profile=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10))

        self.create_command("arm", self.armcommand)
        self.create_command("takeoff", self.takeoff)
        self.create_command("land", self.land)
        self.create_command("rtl", self.rtl_command)
        self.create_command("home", self.home)
        self.create_command("goto", self.goto)
        self.create_command("ddsa", self.ddsa, DDSA)
        self.create_command("lawnmower", self.lawnmower, Lawnmower)
        self.create_command("navigate", self.navigate, Navigation)
        self.create_command("mission", self.mission, Mission)
        self.create_command("setup", self.setup_drone, Setup)
        self.create_command("start_mission", self.start_mission)
        self.create_command("flock", self.mission, Mission)
        self.create_command("cancel", self.cancelCommand)
        self.create_command("hello", self.hello)

        self.node.create_service(DDSAWaypoints, "/build/ddsa", self.build_ddsa)
        self.node.create_service(LawnmowerWaypoints, "/build/lawnmower", self.build_lawnmower)

        print("Setup complete")

        self.boundary_check_thread = threading.Thread(target=self.rtl_boundary_check)
        self.boundary_check_thread.start()


def main():
    rclpy.init(args=sys.argv)

    parser = argparse.ArgumentParser(description='Drone command service.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    node = rclpy.create_node("{}_remote_service".format(args.id))

    command = DragonflyCommand(args.id, node)

    command.setup()

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    command.loop()

    node.destroy_node()
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()
