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

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Arrays;
import java.util.List;

// A driver class for the rest mainly used for testing at the moment
public class Main {

    private static List<Sensor> mRunningSensors;
    private static Lidar timeOfFlight = new Lidar("ttyUSB0", 5807);

    // Rushed
    public static PointCloud pc = new PointCloud();
    private static int reps = 0;
    private static ICP icp;
    private static Transform lastTransform = new Transform();

    static {
        mRunningSensors = List.of(timeOfFlight);
    }

    public static void main(String[] args) throws InterruptedException, IOException {
        mRunningSensors.forEach(Sensor::init);

        PrintWriter fout = new PrintWriter(new FileWriter("in.txt"));

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
//                System.out.printf("%.0f%n", ((float[]) object)[0]); // {angle, distance}
                addPoint((float[]) object);
                fout.printf("%.3f,%.3f%n", ((float[]) object)[0], ((float[]) object)[1]);
            }
        });
        new Thread(client).start();

        client.connect(4000, "127.0.0.1", 5807);
    }




    // very rushed, TODO: un-rush this
    public static void addPoint(float[] object) {
        if ((int) object[0] == 0 && pc.size() >= 10) {
            if (reps == 20) {
                System.out.println("ICP Set");
                icp = new ICP(pc, 5000000);
            }
            if (icp != null) {
                lastTransform = icp.doICP(pc, lastTransform);
                System.out.println(lastTransform);
            }
            pc = new PointCloud();
            reps++;
        }
        pc.addPoint(Point.fromPolar(object));
    }
}
