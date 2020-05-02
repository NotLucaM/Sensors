package com.palyrobotics.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;
import java.util.function.Consumer;

public class PointCloud implements Iterable<Point> {

    private ArrayList<Point> points;

    public PointCloud(Point... points) {
        this.points = new ArrayList<Point>(Arrays.asList(points));
    }

    public PointCloud(ArrayList<Point> points) {
        this.points = points;
    }

    public Point getClosestPoint(Point point) {
        double minDistance = Double.POSITIVE_INFINITY;
        Point closestPoint = null;
        for (Point p: points) {
            if (minDistance > point.distanceTo(p)) { // TODO: determine if < or <= is better in this scenario, guessing it doesnt matter
                minDistance = point.distanceTo(p);
                closestPoint = p;
            }
        }
        return closestPoint;
    }

    public boolean contains(Point point) {
        for (Point p : points) {
            if (point.y == p.y && point.x == p.x) {
                return true;
            }
        }
        return false;
    }

    public int size() {
        return points.size();
    }

    public void addPoint(Point point) {
        points.add(point);
    }

    @Override
    public Iterator<Point> iterator() {
        return points.iterator();
    }

    @Override
    public void forEach(Consumer<? super Point> action) {
        points.forEach(action);
    }
}
