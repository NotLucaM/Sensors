package com.palyrobotics;

import com.palyrobotics.sensors.KumquatVision;
import com.palyrobotics.sensors.Sensor;
import com.palyrobotics.util.ColorConstants;
import org.opencv.core.Scalar;

import java.util.List;

import static com.palyrobotics.processing.VisionProcessing.*;
public class Main {

    private static List<Sensor> mRunningSensors;
    private static KumquatVision camera = new KumquatVision("127.0.0.1", 5802, 5803, true, "Vision");
    private static String order1 = generateOrder(dilate(4), threshold(ColorConstants.kOrangeLowerBoundHSV, ColorConstants.kOrangeUpperBoundHSV), largest(1), "E copy,", "E test,", showContour());
    private static String order2 = generateOrder(erode(2), threshold(new Scalar(1, 2, 3), new Scalar(244, 244 ,244)), largest(1), showContour());

    static {
        camera.setOrder(order2);
        mRunningSensors = List.of(camera);
    }

    public static void main(String[] args) throws InterruptedException {
        mRunningSensors.forEach(Sensor::init);
        Thread.sleep(10000);
        camera.setOrder(order1);
    }
}
