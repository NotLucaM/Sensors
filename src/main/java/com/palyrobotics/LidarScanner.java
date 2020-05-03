package com.palyrobotics;

import com.esotericsoftware.kryonet.Client;
import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.palyrobotics.sensors.Lidar;
import com.palyrobotics.sensors.Sensor;
import com.palyrobotics.util.Point;
import com.palyrobotics.util.PointCloud;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;
import java.util.Scanner;
import java.util.StringTokenizer;

/**
 * A class to make scanning data for the LIDAR more convenient
 */
public class LidarScanner {

    private static List<Sensor> mRunningSensors;
    private static Lidar lidar = new Lidar("cp210x", 5807);


    private static PointCloud lastPointCloud = null;
    private static PointCloud currentPointCloud = new PointCloud();
    private static float lastAngle = 0;

    static {
        mRunningSensors = List.of(lidar);
    }

    public static void main(String[] args) {
        mRunningSensors.forEach(Sensor::init);

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
                }
            }
        });
        new Thread(client).start();

        Scanner in = new Scanner(System.in);
        while (true) {
            var nextString = in.next();
            StringTokenizer command = new StringTokenizer(nextString);

            if (command.countTokens() == 2 && command.nextElement() == "scan") {
                PrintWriter out = null;
                try {
                    out = new PrintWriter(new FileWriter("lidar-scans/" + command.nextElement()));
                } catch (IOException e) {
                    e.printStackTrace();
                }

                var last = lastPointCloud; // Make sure that lastPointCloud is not changed
                for (Point point : last) {
                    assert out != null;
                    out.println(point);
                }
            }
        }
    }

    public static void addPoint(float theta, float r) {
        if (lastAngle > theta) {
            lastPointCloud = currentPointCloud;
            currentPointCloud = new PointCloud();
        }
        currentPointCloud.addPoint(Point.fromPolar(theta, r));
        lastAngle = theta;
    }
}
