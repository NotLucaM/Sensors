package com.palyrobotics.util;

import java.util.Iterator;
import java.util.List;
import java.util.function.Consumer;

public class PointCloud implements Iterable<Point> {

    private List<Point> points;

    public PointCloud(Point... points) {
        this.points = List.of(points);
    }

    public PointCloud(List<Point> points) {
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
