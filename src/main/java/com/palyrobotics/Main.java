package com.palyrobotics;

import com.palyrobotics.processing.VisionProcessing;
import com.palyrobotics.sensors.KumquatVision;
import com.palyrobotics.sensors.Sensor;
import com.palyrobotics.util.ColorConstants;
import com.palyrobotics.processing.VisionProcessing.*;

import java.util.List;

import static com.palyrobotics.processing.VisionProcessing.*;
public class Main {

    private static List<Sensor> mRunningSensors;
    private static KumquatVision camera = new KumquatVision("127.0.0.1", 5802, 5802, 5803, 5803, true, "Vision");
    private static List<Order> order1 = generateOrder(new Erode(4), new Threshold(ColorConstants.kOrangeLowerBoundHSV, ColorConstants.kOrangeUpperBoundHSV), new FindContours(), new Largest(2));
    private static List<Order> order2 = generateOrder(new Threshold(ColorConstants.kOrangeLowerBoundHSV, ColorConstants.kOrangeUpperBoundHSV),
            new FindContours(), new Largest(1), new DrawContours(), new DrawCentroid(ColorConstants.kRedUpperBoundHSV));

    static {
        camera.addActive(new VisionProcessing(order2));
        mRunningSensors = List.of(camera);
    }

    public static void main(String[] args) throws InterruptedException {
        mRunningSensors.forEach(Sensor::init);
        Thread.sleep(10000);
        camera.addActive(new VisionProcessing(order1));
    }
}
