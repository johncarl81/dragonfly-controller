#! /usr/bin/env python
import rospy, time, argparse, math, std_msgs, pulp
from std_srvs.srv import Trigger, TriggerResponse
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from mavros_msgs.msg import State, Waypoint
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import CommandCode, GlobalPositionTarget
from dragonfly_messages.srv import Lawnmower, LawnmowerResponse

def distance(position1, position2):
    x = position1.x - position2.x
    y = position1.y - position2.y
    z = position1.z - position2.z

    return math.sqrt((x * x) + (y * y) + (z * z))

def calculateLatitude(latitude, offset):
    return latitude + (offset * 0.00000898)

def calculateLongitude(latitude, longitude, offset):
    return longitude + (offset * 0.00000898) / math.cos(latitude * 0.01745)

def createWaypoint(lat, lon, altitude, type):
    waypoint = Waypoint()
    waypoint.frame = Waypoint.FRAME_GLOBAL_REL_ALT
    waypoint.command = type
    waypoint.is_current = 0
    waypoint.autocontinue = 0
    waypoint.x_lat = lat
    waypoint.y_long = lon
    waypoint.z_alt = altitude
    waypoint.param1 = 1

    return waypoint

def buildDDSAWaypoints(centerx, centery, altitude, size, index, loops, radius):

    waypoints = []
    start = createWaypoint(centerx, centery, altitude, CommandCode.NAV_WAYPOINT)
    start.is_current = 1
    waypoints.append(start)
    for loop in range(0, loops):
        for corner in range(0, 4):

            if (loop == 0 and corner == 0):
                xoffset = 0
                yoffset = index + 1
            else:
                xoffset = 1 + index + (loop * size)
                yoffset = xoffset
                if (corner == 0):
                    xoffset = -(1 + index + ((loop - 1) * size))
                elif (corner == 3):
                    xoffset = -xoffset
                if (corner == 2 or corner == 3):
                    yoffset = -yoffset

            latitude = calculateLatitude(centerx, xoffset * radius)
            longitude = calculateLongitude(centerx, centery, yoffset * radius)
            waypoint = createWaypoint(latitude, longitude, altitude, CommandCode.NAV_WAYPOINT)
            waypoints.append(waypoint)
    waypoints.append(start)
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

def buildLawnmowerWaypoints(altitude, position, boundary, steplegnth):
    boundary_meters = []

    waypoints = []

    for waypoint in boundary:

        goalPos = PoseStamped()
        goalPos.pose.position.x = (position.longitude - waypoint.longitude) * 111358 * math.cos(position.longitude * 0.01745)
        goalPos.pose.position.y = -(position.latitude - waypoint.latitude) * 111358
        goalPos.pose.position.z = altitude

        waypoints.append(goalPos)

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
        for x in range(int(minx), int(maxx), 1):
            goalPos = PoseStamped()
            goalPos.pose.position.x = x
            goalPos.pose.position.y = y
            goalPos.pose.position.z = altitude
            waypoints.append(goalPos)
        minx = linearXRange(boundary_meters, y + steplegnth, pulp.LpMinimize)
        maxx = linearXRange(boundary_meters, y + steplegnth, pulp.LpMaximize)
        print "minx:{} maxx:{} ".format(minx, maxx)
        for x in range(int(maxx), int(minx), -1):
            goalPos = PoseStamped()
            goalPos.pose.position.x = x
            goalPos.pose.position.y = y + 1
            goalPos.pose.position.z = altitude
            waypoints.append(goalPos)

    return waypoints

class DragonflyCommand:

    def __init__(self, id, altitude, index, swarmsize):
        self.id = id
        self.altitude = altitude
        self.index = index
        self.swarmsize = swarmsize

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
        return TriggerResponse(success=True, message="Commanded {} hello successful.".format(self.id))

    def armcommand(self, operation):
        print "Commanded to arm"

        self.setmode("STABILIZE")

        self.arm()

        self.disarm()

        return TriggerResponse(success=True, message="Commanded {} to arm.".format(self.id))

    def takeoff(self, operation):

        print "Verifying disarmed..."
        def updateState(state):

            if not state.armed:
                print "Commanded to takeoff"

                self.setmode("STABILIZE")

                self.arm()

                self.setmode("GUIDED")

                print "Take off"
                print self.takeoff_service(altitude = self.altitude)
            else:
                print "Takeoff aborted, {} is armed".format(self.id)

            disabled_update.unregister()

        disabled_update = rospy.Subscriber("{}/mavros/state".format(self.id), State, updateState)

        return TriggerResponse(success=True, message="Commanded {} to takeoff.".format(self.id))

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

        return TriggerResponse(success=True, message="Commanded {} to land.".format(self.id))

    def rtl(self, operation):
        print "Commanded to land"

        self.setmode("RTL")

        return TriggerResponse(success=True, message="Commanded {} to rtl.".format(self.id))

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

        return TriggerResponse(success=True, message="Commanded {} to goto.".format(self.id))

    def ddsa(self, operation):
        print "Commanded to ddsa"

        def updatePosition(position):
            position_update.unregister()
            print "Position: ", position.latitude, " ", position.longitude

            rospy.rostime.wallsleep(0.5)

            waypoints = buildDDSAWaypoints(0, 0, self.altitude, self.swarmsize, self.index, 5, 2)

            for waypoint in waypoints:
                rospy.rostime.wallsleep(10)

                goalPos = PoseStamped()
                goalPos.pose.position.x = waypoint.x_lat
                goalPos.pose.position.y = waypoint.y_long
                goalPos.pose.position.z = 5

                print "Going to: ", goalPos

                print self.local_setposition_publisher.publish(goalPos)

                print "Commanded"

        position_update = rospy.Subscriber("{}/mavros/global_position/global".format(self.id), NavSatFix, updatePosition)

        return TriggerResponse(success=True, message="Commanded {} to ddsa.".format(self.id))


    def lawnmower(self, operation):
        print "Commanded to lawnmower"

        zeroPoint = PoseStamped()
        zeroPoint.pose.position.x = 0
        zeroPoint.pose.position.y = 0
        zeroPoint.pose.position.z = operation.altitude

        self.local_setposition_publisher.publish(zeroPoint)
        while(distance(zeroPoint.pose.position, self.position) > 1) :
            rospy.rostime.wallsleep(1)

        print "Zero position"

        def updatePosition(position):
            position_update.unregister()
            print "Position: ", position.latitude, " ", position.longitude

            #rospy.rostime.wallsleep(0.5)

            waypoints = buildLawnmowerWaypoints(operation.altitude, position, operation.boundary, operation.steplength)

            print len(waypoints)
            for waypoint in waypoints:

                # header = std_msgs.msg.Header()
                # header.stamp = rospy.Time.now()
                #
                # goalPos = GlobalPositionTarget()
                # goalPos.header = header
                #
                # goalPos.latitude = waypoint.latitude
                # goalPos.longitude = waypoint.longitude
                # goalPos.altitude = operation.altitude
                #
                # print "Going to: ", goalPos
                #
                # print self.global_setpoint_publisher.publish(goalPos)

                #print "Going to: ", waypoint

                self.local_setposition_publisher.publish(waypoint)

                #print "Commanded"

                while(distance(waypoint.pose.position, self.position) > 1) :
                    #print "Distance to point: ", distance(waypoint.pose.position, self.position)
                    rospy.rostime.wallsleep(1)
                print "{}, {}".format(waypoint.pose.position.x, waypoint.pose.position.y)
                #rospy.rostime.wallsleep(10)

            goalPos = PoseStamped()
            goalPos.pose.position.x = 0
            goalPos.pose.position.y = 0
            goalPos.pose.position.z = operation.altitude

            print "Going to: ", goalPos

            print self.local_setposition_publisher.publish(goalPos)

        position_update = rospy.Subscriber("{}/mavros/global_position/global".format(self.id), NavSatFix, updatePosition)

        return LawnmowerResponse(success=True, message="Commanded {} to lawnmower.".format(self.id))

    def position(self, data):
        self.position = data.pose.position

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
        self.global_setpoint_publisher = rospy.Publisher("{}/mavros/setpoint_position/global".format(self.id), GlobalPositionTarget, queue_size=1)

        rospy.Subscriber("{}/mavros/local_position/pose".format(self.id), PoseStamped, self.position)



        rospy.Service("/{}/command/arm".format(self.id), Trigger, self.armcommand)
        rospy.Service("/{}/command/takeoff".format(self.id), Trigger, self.takeoff)
        rospy.Service("/{}/command/land".format(self.id), Trigger, self.land)
        rospy.Service("/{}/command/rtl".format(self.id), Trigger, self.rtl)
        rospy.Service("/{}/command/goto".format(self.id), Trigger, self.goto)
        rospy.Service("/{}/command/ddsa".format(self.id), Trigger, self.ddsa)
        rospy.Service("/{}/command/lawnmower".format(self.id), Lawnmower, self.lawnmower)
        rospy.Service("/{}/command/hello".format(self.id), Trigger, self.hello)

        print "Setup complete"

        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Drone command service.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    parser.add_argument('alt', type=int, help='Altitude to fly.')
    parser.add_argument('index', type=int, help='Index in swarm.')
    parser.add_argument('size', type=int, help='Swarm size.')
    args = parser.parse_args()

    command = DragonflyCommand(args.id, args.alt, args.index, args.size)

    command.setup()

