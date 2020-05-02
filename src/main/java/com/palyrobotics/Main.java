package com.palyrobotics;

import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.palyrobotics.sensors.Lidar;
import com.palyrobotics.sensors.Sensor;
import com.palyrobotics.sensors.TimeOfFlightSensor;
import com.esotericsoftware.kryonet.Client;

import java.io.IOException;
import java.util.Arrays;
import java.util.List;

// A driver class for the rest mainly used for testing at the moment
public class Main {

    private static List<Sensor> mRunningSensors;
    private static Lidar timeOfFlight = new Lidar("ttyUSB0", 5807);

    static {
        mRunningSensors = List.of(timeOfFlight);
    }

    public static void main(String[] args) throws InterruptedException, IOException {
        mRunningSensors.forEach(Sensor::init);

        // Very rushed way to test out ICP
        Client client = new Client();
        client.getKryo().register(float[].class);
        client.addListener(new Listener() {
            @Override
            public void connected(Connection connection) {
                System.out.println("Connected");
            }

            @Override
            public void received(Connection connection, Object object) {
                System.out.printf("%.0f%n", ((float[]) object)[0]);
            }
        });
        new Thread(client).start();

        client.connect(4000, "127.0.0.1", 5807);
    }



    // very rushed, TODO: un-rush this
    public static void addPoint(float[] object) {

    }
}
