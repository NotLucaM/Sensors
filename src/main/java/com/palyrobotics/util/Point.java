package com.palyrobotics.util;

public class Point {

    double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distanceTo(Point p) {
        return Math.sqrt(Math.pow(x - p.x, 2) + Math.pow(y - p.y, 2));
    }

    public static Point fromPolar(float[] point) {
        return new Point(point[1] * Math.cos(Math.toRadians(point[0])), point[1] * Math.sin(Math.toRadians(point[0])));
    }

    public static Point fromPolar(double[] point) {
        return new Point(point[1] * Math.cos(Math.toRadians(point[0])), point[1] * Math.sin(Math.toRadians(point[0])));
    }

    public static Point fromPolar(double thetaDegrees, double r) {
        return new Point(r * Math.cos(Math.toRadians(thetaDegrees)), r * Math.sin(Math.toRadians(thetaDegrees)));
    }

    @Override
    public String toString() {
        return String.format("%f %f", x, y);
    }
}
