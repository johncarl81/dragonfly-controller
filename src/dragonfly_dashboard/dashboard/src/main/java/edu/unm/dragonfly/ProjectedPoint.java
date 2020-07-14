package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;

public class ProjectedPoint {

    private final Point original;
    private final double x;
    private final double y;
    private final double z;

    public ProjectedPoint(Point point) {
        this.original = point;
        this.x = point.getX() * 111358 * Math.cos(point.getX() * 0.01745);
        this.y = point.getY() * 111358;
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
