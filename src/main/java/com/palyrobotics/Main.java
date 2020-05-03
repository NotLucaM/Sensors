package com.palyrobotics;

import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.palyrobotics.sensors.Lidar;
import com.palyrobotics.sensors.Sensor;
import com.palyrobotics.sensors.TimeOfFlightSensor;
import com.esotericsoftware.kryonet.Client;
import com.palyrobotics.util.ICP;
import com.palyrobotics.util.Point;
import com.palyrobotics.util.PointCloud;
import com.palyrobotics.util.Transform;

import java.io.*;
import java.util.Arrays;
import java.util.List;

// A driver class for the rest mainly used for testing at the moment
public class Main {

    private static List<Sensor> mRunningSensors;
    private static Lidar timeOfFlight = new Lidar("ttyUSB0", 5807);

    // Rushed
    public static PointCloud pc = new PointCloud();
    private static ICP icp;
    private static Transform lastTransform = new Transform();
    private static float lastAngle = 0;
    private static long timeout = 10000000; // 10ms ik

    static {
        mRunningSensors = List.of(timeOfFlight);
    }

    public static void main(String[] args) throws InterruptedException, IOException {
        mRunningSensors.forEach(Sensor::init);

//        PrintWriter fout = new PrintWriter(new FileWriter("out.txt"));

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
                if (object instanceof float[]) { // Java 14 when?
                    addPoint(((float[]) object)[0], ((float[]) object)[1]);
//                    fout.printf("%f,%f%n", ((float[]) object)[0], ((float[]) object)[1]);
                }
            }
        });
        new Thread(client).start();

        BufferedReader bf = new BufferedReader(new FileReader("out.txt"));
        PointCloud reference = new PointCloud();
        while (true) {
            String line = bf.readLine();

            if (line == null) {
                break;
            }

            var split = line.split(",");
            reference.addPoint(Point.fromPolar(new float[]{Float.parseFloat(split[0]),
                                                            Float.parseFloat(split[1])}));
        }
        icp = new ICP(reference, timeout);

        client.connect(4000, "127.0.0.1", 5807);
    }

    // very rushed, TODO: un-rush this
    public static void addPoint(float theta, float r) {
        if (lastAngle > theta) {
            lastTransform = icp.doICP(pc, lastTransform);
            System.out.println(lastTransform);
            pc = new PointCloud();
        }
        pc.addPoint(Point.fromPolar(theta, r));
        lastAngle = theta;
    }
}
