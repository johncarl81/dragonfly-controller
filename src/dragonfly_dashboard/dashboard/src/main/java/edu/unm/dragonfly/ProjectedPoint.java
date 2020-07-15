package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;

public class ProjectedPoint {

    private static final double EARTH_CIRCUMFERENCE_METERS = 40008000;

    private final Point original;
    private final double x;
    private final double y;
    private final double z;

    public ProjectedPoint(Point point) {
        this.original = point;
        double latitudeCircumfrence = EARTH_CIRCUMFERENCE_METERS * Math.cos(point.getY() * Math.PI / 180);
        this.x = point.getX() * latitudeCircumfrence / 360;
        this.y = point.getY() * EARTH_CIRCUMFERENCE_METERS / 360;
        this.z = point.getZ();
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public Point getOriginal() {
        return original;
    }
}
