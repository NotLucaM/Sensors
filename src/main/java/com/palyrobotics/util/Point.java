package com.palyrobotics.util;

public class Point {

    double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distanceTo(Point p) {
        return Math.sqrt(Math.pow(x + p.x, 2) + Math.pow(y + p.y, 2));
    }
}
