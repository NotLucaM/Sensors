package com.palyrobotics;

import com.palyrobotics.sensors.Lidar;
import com.palyrobotics.sensors.Sensor;

import java.io.IOException;
import java.util.List;

// A driver class for the rest mainly used for testing at the moment
public class LidarServer {

    private static List<Sensor> mRunningSensors;
    private static Lidar lidar = new Lidar("cp210x", 5807);

    static {
        mRunningSensors = List.of(lidar);
    }

    public static void main(String[] args) {
        mRunningSensors.forEach(Sensor::init);
    }
}
