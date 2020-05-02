package com.palyrobotics.util;

public class ICP {

    private PointCloud reference; // The field
    private long timeout; // Timeout for each call of doICP in nano seconds

    public Transform doICP(PointCloud lidarOutput) {
        return doICP(lidarOutput, new Transform());
    }

    public Transform doICP(PointCloud lidarOutput, Transform transform) {
        long startingTime = System.nanoTime();

        while (startingTime - System.nanoTime() <= timeout) {

        }
        return null;
    }
}
