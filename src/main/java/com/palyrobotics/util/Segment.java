package com.palyrobotics.util;

public class Segment {

    Point p1, p2;

    public Segment(Point p1, Point p2) {
        this.p1 = p1;
        this.p2 = p2;
    }

    //https://stackoverflow.com/questions/30559799/function-for-finding-the-distance-between-a-point-and-an-edge-in-java
    public double getDistance(Point p) {
        double deltaX = p.x - p1.x;
        double deltaY = p.y - p1.y;
        double deltaPX = p2.x - p1.x;
        double deltaPY = p2.y - p1.y;
        double orthogonal = -deltaPY;

        double dot = deltaX * orthogonal + deltaY * deltaPX;
        double lenSquared = orthogonal * orthogonal + deltaPX * deltaPX;

        return Math.abs(dot) / Math.sqrt(lenSquared);
    }
}
