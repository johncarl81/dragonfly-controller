package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;

public class NavigateWaypoint {

    private final Point point;
    private final float distanceThreshold;

    public NavigateWaypoint(Point point, float distanceThreshold) {
        this.point = point;
        this.distanceThreshold = distanceThreshold;
    }

    public Point getPoint() {
        return point;
    }

    public float getDistanceThreshold() {
        return distanceThreshold;
    }
}
