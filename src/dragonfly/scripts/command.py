#! /usr/bin/env python
import rospy, time, argparse, math, std_msgs, pulp
from enum import Enum
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import String
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State, Waypoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point
from dragonfly_messages.msg import LatLon
from dragonfly_messages.srv import Lawnmower, LawnmowerResponse, LawnmowerWaypoints, LawnmowerWaypointsResponse, DDSA, DDSAResponse, DDSAWaypoints, DDSAWaypointsResponse, Navigation, NavigationResponse

class Span(Enum):
    WALK = 1
    RANGE = 2

def distance(position1, position2):
    deltax = position1.x - position2.x
    deltay = position1.y - position2.y
    deltaz = position1.z - position2.z

    return math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))

def createWaypoint(x, y, altitude):
    waypoint = PoseStamped()
    waypoint.pose.position.x = x
    waypoint.pose.position.y = y
    waypoint.pose.position.z = altitude

    return waypoint

def calculateRange(type, start, end, length):
    print "TYPE: {} {} {}".format(type, Span.WALK, Span.RANGE)
    if type == Span.WALK:
        waypoints = []
        print "Calculating walk"
        deltax = end.x - start.x
        deltay = end.y - start.y
        deltaz = end.z - start.z
        distance = math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))
        for i in range(1, int(distance / length) + 1):
            waypoints.append(Point(start.x + (i * length * deltax / distance),
                                   start.y + (i * length * deltay / distance),
                                   start.z + (i * length * deltaz / distance)))
        return waypoints
    elif type == Span.RANGE:
        return [end]

def buildRelativeWaypoint(localx, localy, positionLon, positionLat, waypointLon, waypointLat, altitude):
    earthCircumference = 40008000
    return createWaypoint(
        localx - ((positionLon - waypointLon) * (earthCircumference / 360) * math.cos(positionLat * 0.01745)),
        localy - ((positionLat - waypointLat) * (earthCircumference / 360)) ,
        altitude
    )

def createLatLon(localwaypoint, localposition, position):
    earthCircumference = 40008000
    latitude = position.latitude - (localposition.y - localwaypoint.y) * 360 / earthCircumference
    longitude= position.longitude - (localposition.x - localwaypoint.x) * 360 / (earthCircumference * math.cos(latitude * 0.01745))

    return LatLon(latitude = latitude, longitude = longitude, relativeAltitude = localwaypoint.z)

def build3DDDSAWaypoints(rangeType, stacks, size, index, loops, radius, steplength):
    waypoints = []
    toggleReverse = False
    for stack in range(0, stacks):

        ddsaWaypoints = buildDDSAWaypoints(rangeType, stack, size, index, loops, radius, steplength)
        if toggleReverse:
            ddsaWaypoints = ddsaWaypoints[::-1]
        waypoints = waypoints + ddsaWaypoints

        toggleReverse = not toggleReverse

    return waypoints

def buildDDSAWaypoints(rangeType, altitude, size, index, loops, radius, steplength):

    waypoints = []
    start = Point(0, 0, altitude)
    waypoints.append(start)
    previous = start
    for loop in range(0, loops):
        for corner in range(0, 4):

            if (loop == 0 and corner == 0):
                next = Point(0, index + 1, altitude)
            else:
                xoffset = 1 + index + (loop * size)
                yoffset = xoffset
                if (corner == 0):
                    xoffset = -(1 + index + ((loop - 1) * size))
                elif (corner == 3):
                    xoffset = -xoffset
                if (corner == 2 or corner == 3):
                    yoffset = -yoffset

                next = Point(xoffset, yoffset, altitude)

            print "{}, {} -> {}, {}".format(previous.x, previous.y, next.x, next.y)

            for waypoint in calculateRange(rangeType, previous, next, steplength):
                waypoints.append(Point(waypoint.x * radius, waypoint.y * radius, waypoint.z))

            previous = next

    return waypoints

def linearXRange(points, setY, type):

    problem = pulp.LpProblem('range', type)

    x = pulp.LpVariable('x', cat='Continuous')
    y = pulp.LpVariable('y', cat='Continuous')

    # Objective function
    problem += x

    def buildLineEquation(index1, index2):
        a = -(points[index2][1] - points[index1][1])
        b = points[index2][0] - points[index1][0]
        c = (a * points[index1][0]) + (b * points[index1][1])
        # print '(', a, 'x+',b,'y >=',c,'),'
        return (a * x) + (b * y) >= c

    for i in range(1, len(points)):
        problem +=buildLineEquation(i-1, i)

    problem += buildLineEquation(len(points)-1, 0)

    problem += y == setY

    # print problem
    pulp.GLPK_CMD(msg=0).solve(problem)

    return x.value()

def linearYRange(points, type):

    problem = pulp.LpProblem('range', type)

    x = pulp.LpVariable('x', cat='Continuous')
    y = pulp.LpVariable('y', cat='Continuous')

    # Objective function
    problem += y

    def buildLineEquation(index1, index2):
        a = -(points[index2][1] - points[index1][1])
        b = points[index2][0] - points[index1][0]
        c = (a * points[index1][0]) + (b * points[index1][1])
        # print '(', a, 'x+',b,'y >=',c,'),'
        return (a * x) + (b * y) >= c

    for i in range(1, len(points)):
        problem +=buildLineEquation(i-1, i)

    problem += buildLineEquation(len(points)-1, 0)

    # print problem
    pulp.GLPK_CMD(msg=0).solve(problem)

    return y.value()

def build3DLawnmowerWaypoints(rangeType, altitude, localPosition, position, stacks, boundary, steplength):
    waypoints = []
    toggleReverse = False
    for stack in range(0, stacks):

        lawnmowerWaypoints = buildLawnmowerWaypoints(rangeType, altitude + stack, localPosition, position, boundary, steplength)
        if toggleReverse:
            lawnmowerWaypoints = lawnmowerWaypoints[::-1]
        waypoints = waypoints + lawnmowerWaypoints

        toggleReverse = not toggleReverse

    return waypoints

def buildLawnmowerWaypoints(rangeType, altitude, localposition, position, boundary, steplegnth):
    boundary_meters = []

    waypoints = []

    for waypoint in boundary:
        goalPos = buildRelativeWaypoint(localposition.x, localposition.y, position.longitude, position.latitude, waypoint.longitude, waypoint.latitude, altitude)

        boundary_meters.append((goalPos.pose.position.x, goalPos.pose.position.y))


    # Get minimum in Y dimension
    miny = linearYRange(boundary_meters, pulp.LpMinimize)
    # Get maximum in Y dimension
    maxy = linearYRange(boundary_meters, pulp.LpMaximize)


    print "miny:{} maxy:{} ".format(miny, maxy)

    for y in range(int(miny), int(maxy), 2):
        minx = linearXRange(boundary_meters, y, pulp.LpMinimize)
        maxx = linearXRange(boundary_meters, y, pulp.LpMaximize)
        print "minx:{} maxx:{} ".format(minx, maxx)
        waypoints.append(createWaypoint(minx, y, altitude))
        for point in calculateRange(rangeType, Point(minx, y, altitude), Point(maxx, y, altitude), steplegnth):
            waypoints.append(createWaypoint(point.x, point.y, point.z))
        minx = linearXRange(boundary_meters, y + steplegnth, pulp.LpMinimize)
        maxx = linearXRange(boundary_meters, y + steplegnth, pulp.LpMaximize)
        print "minx:{} maxx:{} ".format(minx, maxx)
        waypoints.append(createWaypoint(maxx, y + steplegnth, altitude))
        for point in calculateRange(rangeType, Point(maxx, y + steplegnth, altitude), Point(minx, y + steplegnth, altitude), steplegnth):
            waypoints.append(createWaypoint(point.x, point.y, point.z))

    return waypoints

class DragonflyCommand:

    def __init__(self, id):
        self.zeroing = False
        self.canceled = False
        self.sincezero = 0
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

    def hello(self, command):
        print "hello"
        return EmptyResponse()

    def armcommand(self, operation):
        print "Commanded to arm"

        self.setmode("STABILIZE")

        self.arm()

        self.disarm()

        return EmptyResponse()

    def takeoff(self, operation):

        print "Verifying disarmed..."
        def updateState(state):

            if not state.armed:
                print "Commanded to takeoff"

                self.setmode("STABILIZE")

                self.arm()

                self.setmode("GUIDED")

                print "Take off"
                print self.takeoff_service(altitude = 10)
            else:
                print "Takeoff aborted, {} is armed".format(self.id)

            disabled_update.unregister()

        disabled_update = rospy.Subscriber("{}/mavros/state".format(self.id), State, updateState)

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

    def build_ddsa(self, operation):
        ddsaWaypoints = build3DDDSAWaypoints(Span(operation.walk),operation.stacks, 1, 0, operation.loops, operation.radius, operation.steplength)

        localWaypoints = []
        for localwaypoint in ddsaWaypoints:
            localWaypoints.append(createWaypoint(self.localposition.x + localwaypoint.x, self.localposition.y + localwaypoint.y, operation.altitude + localwaypoint.z))

        waypoints = []
        for localwaypoint in localWaypoints:
            waypoints.append(createLatLon(localwaypoint.pose.position, self.localposition, self.position))

        return DDSAWaypointsResponse(waypoints=waypoints)

    def ddsa(self, operation):
        print "Commanded to ddsa"
        self.logPublisher.publish("DDSA started")

        self.canceled = False
        self.setmode('GUIDED')

        ddsaWaypoints = build3DDDSAWaypoints(Span(operation.walk),operation.stacks, 1, 0, operation.loops, operation.radius, operation.steplength)

        waypoints = []
        for localwaypoint in ddsaWaypoints:
            waypoints.append(createWaypoint(self.localposition.x + localwaypoint.x, self.localposition.y + localwaypoint.y, operation.altitude + localwaypoint.z))

        self.runWaypoints(waypoints, operation.waittime)

        self.logPublisher.publish("DDSA Finished")

        return DDSAResponse(success=True, message="Commanded {} to DDSA.".format(self.id))

    def build_lawnmower(self, operation):

        lawnmowerLocalWaypoints = []

        if operation.walkBoundary:
            wrappedGPSBoundary = []
            wrappedGPSBoundary.extend(operation.boundary)
            wrappedGPSBoundary.append(operation.boundary[0])

            for waypoint in wrappedGPSBoundary:
                lawnmowerLocalWaypoints.append(buildRelativeWaypoint(self.localposition.x, self.localposition.y, self.position.longitude, self.position.latitude, waypoint.longitude, waypoint.latitude, operation.altitude))

        lawnmowerLocalWaypoints.extend(build3DLawnmowerWaypoints(Span(operation.walk), operation.altitude, self.localposition, self.position, operation.stacks, operation.boundary, operation.steplength))

        waypoints = []
        for lawnmowerLocalWaypoint in lawnmowerLocalWaypoints:
            waypoints.append(createLatLon(lawnmowerLocalWaypoint.pose.position, self.localposition, self.position))

        return LawnmowerWaypointsResponse(waypoints=waypoints)

    def lawnmower(self, operation):
        print "Commanded to lawnmower"
        self.logPublisher.publish("Lawnmower started")

        self.canceled = False
        self.setmode('GUIDED')

        print "Position: {} {} {}".format(self.localposition.x, self.localposition.y, self.localposition.z)

        waypoints = []

        if operation.walkBoundary:
            print "Walking boundary"

            wrappedGPSBoundary = operation.boundary
            wrappedGPSBoundary.append(operation.boundary[0])

            for waypoint in wrappedGPSBoundary:
                waypoints.append(buildRelativeWaypoint(self.localposition.x, self.localposition.y, self.position.longitude, self.position.latitude, waypoint.longitude, waypoint.latitude, operation.altitude))

        waypoints.extend(build3DLawnmowerWaypoints(Span(operation.walk), operation.altitude, self.localposition, self.position, operation.stacks, operation.boundary, operation.steplength))

        self.runWaypoints(waypoints, operation.waittime)

        self.logPublisher.publish("Lawnmower Finished")

        return LawnmowerResponse(success=True, message="Commanded {} to lawnmower.".format(self.id))

    def navigate(self, operation):
        print "Commanded to navigate"
        self.logPublisher.publish("Navigation started")

        self.canceled = False
        self.setmode('GUIDED')

        print "{} {}".format(self.localposition.z, self.position.altitude)

        localWaypoints = []
        for waypoint in operation.waypoints:
            print "{} {} {}".format(self.localposition.z, self.position.altitude, waypoint.relativeAltitude)
            localWaypoints.append(buildRelativeWaypoint(self.localposition.x, self.localposition.y, self.position.longitude, self.position.latitude, waypoint.longitude, waypoint.latitude, waypoint.relativeAltitude))

        self.runWaypoints(localWaypoints, operation.waittime)

        self.logPublisher.publish("Navigation Finished")

        return NavigationResponse(success=True, message="Commanded {} to navigate.".format(self.id))

    def runWaypoints(self, waypoints, waittime):

        i = 0
        for waypoint in waypoints:
            self.local_setposition_publisher.publish(waypoint)

            print "Distance to point:{} {} {}".format(waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z), distance(waypoint.pose.position, self.localposition)
            while not self.canceled and (distance(waypoint.pose.position, self.localposition) > 1 or self.zeroing) :
                print "Distance to point: ", distance(waypoint.pose.position, self.localposition)
                rospy.rostime.wallsleep(1)
            self.logPublisher.publish("Navigation at {}, {}, {}, {}".format(i, waypoint.pose.position.x, waypoint.pose.position.y, waypoint.pose.position.z))
            rospy.rostime.wallsleep(waittime)
            i = i+1

            if self.canceled:
                self.logPublisher.publish("Navigation canceled")
                break

        self.logPublisher.publish("Navigation Finished")
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

    def setup(self):
        rospy.init_node("{}_remote_service".format(self.id))

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

        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Drone command service.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    # parser.add_argument('alt', type=int, help='Altitude to fly.')
    # parser.add_argument('index', type=int, help='Index in swarm.')
    # parser.add_argument('size', type=int, help='Swarm size.')
    args = parser.parse_args()

    command = DragonflyCommand(args.id)

    command.setup()

