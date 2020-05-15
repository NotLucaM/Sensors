package com.palyrobotics.util;

import org.junit.Test;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class ICPTest {

    @Test
    public void doICP() throws IOException {
        BufferedReader bf = new BufferedReader(new FileReader("test1cycle.txt"));

        PointCloud reference = new PointCloud();
        PointCloud pc = new PointCloud();

        for (int i = 1; i <= 84; i++) {
            String line = bf.readLine();

            if (line == null) {
                break;
            }

            var split = line.split(",");
            reference.addPoint(new Point(Float.parseFloat(split[0]),
                                         Float.parseFloat(split[1])));

            Point p = new Point(Float.parseFloat(split[0]) - 50,
                              Float.parseFloat(split[1]) - 10);

            pc.addPoint(p);
        }

        long timeout = 10000000; // for debugging
        ICP icp = new ICP(timeout, new HashMap<>(Map.of(reference, new Transform())));
        System.out.println(icp.doICP(pc, new Transform(0, -40, -4))); // The reason you can give it a Transform is because you most likely will know your last position
    }
}