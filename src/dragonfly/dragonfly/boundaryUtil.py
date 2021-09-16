#!/usr/bin/env python

def isInside(point, boundary):
    def isLeft(a, b, c):
        return ((b.longitude - a.longitude) * (c.latitude - a.latitude) - (b.latitude - a.latitude) * (
                    c.longitude - a.longitude)) > 0

    for i in range(len(boundary) - 1):
        if not isLeft(boundary[i], boundary[i + 1], point):
            return False

    return isLeft(boundary[len(boundary) - 1], boundary[0], point)
