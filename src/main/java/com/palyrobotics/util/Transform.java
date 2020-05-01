package com.palyrobotics.util;

public class Transform {

    public final double theta, tx, ty;

    public Transform() {
        theta = 0;
        tx = ty = 0;
    }

    // Theta is in radians obviously cus im not savage also cus im lazy and dont want to convert to radians each time i use math
    public Transform(double theta, double tx, double ty) {
        this.theta = theta;
        this.tx = tx;
        this.ty = ty;
    }

    // The following functions apply the transform onto various classes
    public Point apply(Point t) {
        return new Point(t.x * Math.cos(theta)- t.y * Math.sin(theta),
                        t.x * Math.sin(theta) - t.y * Math.cos(theta));
    }

//    public PointCloud apply(PointCloud t) {
//    }

    public Transform inverse() {
        return new Transform(-theta,
                -tx * Math.cos(theta) - ty * Math.sin(theta),
                tx * Math.sin(theta) - ty * Math.cos(theta));
    }
}
