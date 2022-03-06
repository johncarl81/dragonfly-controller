#!/usr/bin/env python
import math
from enum import Enum

import pulp
from geometry_msgs.msg import PoseStamped, Point

from dragonfly_messages.msg import LatLon


class Span(Enum):
    WALK = 1
    RANGE = 2


def createWaypoint(x, y, altitude, orientation):
    waypoint = PoseStamped()
    waypoint.pose.position.x = float(x)
    waypoint.pose.position.y = float(y)
    waypoint.pose.position.z = float(altitude)
    waypoint.pose.orientation.z = float(orientation.z)
    waypoint.pose.orientation.w = float(orientation.w)

    return waypoint


def calculateRange(type, start, end, length):
    if type == Span.WALK:
        waypoints = []
        deltax = end.x - start.x
        deltay = end.y - start.y
        deltaz = end.z - start.z
        distance = math.sqrt((deltax * deltax) + (deltay * deltay) + (deltaz * deltaz))
        for i in range(1, int(distance / length) + 1):
            waypoints.append(Point(x=start.x + (i * length * deltax / distance),
                                   y=start.y + (i * length * deltay / distance),
                                   z=start.z + (i * length * deltaz / distance)))
        return waypoints
    elif type == Span.RANGE:
        return [end]


def buildRelativeWaypoint(localposition, position, waypoint, altitude, orientation):
    earthCircumference = 40008000
    return createWaypoint(
        localposition.x - ((position.longitude - waypoint.longitude) * (earthCircumference / 360) * math.cos(
            position.latitude * 0.01745)),
        localposition.y - ((position.latitude - waypoint.latitude) * (earthCircumference / 360)),
        altitude,
        orientation
    )


def createLatLon(localwaypoint, localposition, position):
    earthCircumference = 40008000
    latitude = position.latitude - (localposition.y - localwaypoint.y) * 360 / earthCircumference
    longitude = position.longitude - (localposition.x - localwaypoint.x) * 360 / (
            earthCircumference * math.cos(latitude * 0.01745))

    return LatLon(latitude=latitude, longitude=longitude, relative_altitude=localwaypoint.z)


def build3DDDSAWaypoints(rangeType, stacks, size, index, loops, radius, step_length):
    waypoints = []
    toggleReverse = False
    for stack in range(0, stacks):

        ddsaWaypoints = buildDDSAWaypoints(rangeType, stack, size, index, loops, radius, step_length)
        if toggleReverse:
            ddsaWaypoints = ddsaWaypoints[::-1]
        waypoints = waypoints + ddsaWaypoints

        toggleReverse = not toggleReverse

    return waypoints


def buildDDSAWaypoints(rangeType, altitude, size, index, loops, radius, step_length):
    waypoints = []
    start = Point(x=-float(index * radius), y=0.0, z=float(altitude))
    waypoints.append(start)
    previous = start
    for loop in range(loops):
        for corner in range(4):

            xoffset = loop * size + index + 1
            yoffset = xoffset
            if corner == 0:
                xoffset = -size * loop - index
            if corner == 2 or corner == 3:
                yoffset = -yoffset
            if corner == 3:
                xoffset = -xoffset - (size - 1)
                # Ends loop square with the last corner
                if loop == loops - 1:
                    xoffset += index + 1

            next = Point(x=float(xoffset), y=float(yoffset), z=float(altitude))

            for waypoint in calculateRange(rangeType, previous, next, step_length):
                waypoints.append(Point(x=float(waypoint.x * radius), y=float(waypoint.y * radius), z=float(waypoint.z)))

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
        # print('(', a, 'x+',b,'y >=',c,'),')
        return (a * x) + (b * y) >= c

    for i in range(1, len(points)):
        problem += buildLineEquation(i - 1, i)

    problem += buildLineEquation(len(points) - 1, 0)

    problem += y == setY

    # print problem
    pulp.GLPK_CMD(msg=False).solve(problem)

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
        # print('(', a, 'x+',b,'y >=',c,'),')
        return (a * x) + (b * y) >= c

    for i in range(1, len(points)):
        problem += buildLineEquation(i - 1, i)

    problem += buildLineEquation(len(points) - 1, 0)

    # print problem
    pulp.GLPK_CMD(msg=False).solve(problem)

    return y.value()


def build3DLawnmowerWaypoints(rangeType, altitude, localPosition, position, stacks, boundary, step_length, orientation):
    waypoints = []
    toggleReverse = False
    for stack in range(0, stacks):

        lawnmowerWaypoints = buildLawnmowerWaypoints(rangeType, altitude + stack, localPosition, position, boundary,
                                                     step_length, orientation)
        if toggleReverse:
            lawnmowerWaypoints = lawnmowerWaypoints[::-1]
        waypoints = waypoints + lawnmowerWaypoints

        toggleReverse = not toggleReverse

    return waypoints


def buildLawnmowerWaypoints(rangeType, altitude, localposition, position, boundary, step_length, orientation):
    boundary_meters = []
    altitude = float(altitude)
    waypoints = []

    for waypoint in boundary:
        goalPos = buildRelativeWaypoint(localposition, position, waypoint, altitude, orientation)

        boundary_meters.append((goalPos.pose.position.x, goalPos.pose.position.y))

    # Get minimum in Y dimension
    miny = linearYRange(boundary_meters, pulp.LpMinimize)
    # Get maximum in Y dimension
    maxy = linearYRange(boundary_meters, pulp.LpMaximize)

    print("miny:{} maxy:{} ".format(miny, maxy))

    stepdirection = 1 if miny < maxy else -1

    for y in range(int(math.ceil(miny)), int(math.floor(maxy)), int(2 * step_length)):
        minx = float(linearXRange(boundary_meters, y, pulp.LpMinimize))
        maxx = float(linearXRange(boundary_meters, y, pulp.LpMaximize))
        print("minx:{} maxx:{} ".format(minx, maxx))
        waypoints.append(createWaypoint(minx, y, altitude, orientation))
        for point in calculateRange(rangeType, Point(x=float(minx), y=float(y), z=float(altitude)), Point(x=float(maxx), y=float(y), z=float(altitude)),
                                    step_length):
            waypoints.append(createWaypoint(point.x, point.y, point.z, orientation))
        minx = float(linearXRange(boundary_meters, y + step_length, pulp.LpMinimize))
        maxx = float(linearXRange(boundary_meters, y + step_length, pulp.LpMaximize))
        print("minx:{} maxx:{} ".format(minx, maxx))
        waypoints.append(createWaypoint(maxx, y + step_length, altitude, orientation))
        for point in calculateRange(rangeType, Point(x=float(maxx), y=float(y + (stepdirection * step_length)), z=float(altitude)),
                                    Point(x=float(minx), y=float(y + (stepdirection * step_length)), z=float(altitude)), step_length):
            waypoints.append(createWaypoint(point.x, point.y, point.z, orientation))

    return waypoints
