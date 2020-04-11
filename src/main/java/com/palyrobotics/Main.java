package com.palyrobotics;

import com.palyrobotics.sensors.Sensor;
import com.palyrobotics.sensors.TimeOfFlightSensor;

import java.util.List;

public class Main {

    private static List<Sensor> mRunningSensors;

    static {
        mRunningSensors = List.of(new TimeOfFlightSensor("input"));
    }

    public static void main(String[] args) throws InterruptedException {
        mRunningSensors.forEach(Sensor::init);

        ((TimeOfFlightSensor) mRunningSensors.get(0)).searchForPorts();
    }
}
