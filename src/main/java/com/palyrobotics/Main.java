package com.palyrobotics;

import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.palyrobotics.sensors.Lidar;
import com.palyrobotics.sensors.Sensor;
import com.palyrobotics.sensors.TimeOfFlightSensor;
import com.esotericsoftware.kryonet.Client;

import java.io.IOException;
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


        Client client = new Client();
        client.addListener(new Listener() {
            @Override
            public void connected(Connection connection) {
                System.out.println("Connected");
            }

            @Override
            public void received(Connection connection, Object object) {
                System.out.println(object);
            }
        });
        new Thread(client).start();

        client.connect(4000, "127.0.0.1", 5807);
    }
}
