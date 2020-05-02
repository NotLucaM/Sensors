package com.palyrobotics.util;

import org.junit.Test;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;

public class ICPTest {

    @Test
    public void doICP() throws IOException {
        BufferedReader bf = new BufferedReader(new FileReader("test1cycle.txt"));

        PointCloud reference = new PointCloud();
        PointCloud pc = new PointCloud();

        while (true) {
            String line = bf.readLine();

            if (line == null) {
                break;
            }

            var split = line.split(",");
            reference.addPoint(Point.fromPolar(new float[]{Float.parseFloat(split[0]),
                                                    Float.parseFloat(split[1])}));

            var p = Point.fromPolar(new float[]{Float.parseFloat(split[0]),
                    Float.parseFloat(split[1])});

            pc.addPoint(p);
        }

        long timeout = 100000; // for debugging
        ICP icp = new ICP(reference, timeout);
        System.out.println(icp.doICP(pc));
    }
}