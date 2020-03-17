package com.palyrobotics.processing;

import com.palyrobotics.util.ColorConstants;
import com.palyrobotics.sensors.KumquatVision;
import com.palyrobotics.util.Range;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;
import java.util.function.Function;
import java.util.stream.Collectors;

public class VisionProcessing {

    private ArrayList<Function> mOrders;
    private double mImageSize = KumquatVision.kCaptureHeight * KumquatVision.kCaptureWidth;

    public VisionProcessing() {
    }

    public VisionProcessing(String orders) {
        setOrders(orders);
    }

    public static String generateOrder(String... orders) {
        StringBuilder builder = new StringBuilder();
        for (var order : orders) {
            builder.append(order);
        }
        return builder.toString();
    }

    // TODO: use Jackson to store in json

    private class Blur implements Function<Mat, Mat> {

        private Size mSize;
        private Mat mTempMat;

        public Blur(int intensity) {
            setIntensity(intensity);
            this.mTempMat = new Mat();
        }

        @Override
        public Mat apply(Mat mat) {
            Imgproc.blur(mat, mTempMat, mSize);
            return mTempMat;
        }

        public void setIntensity(int intensity) {
            mSize = new Size(intensity, intensity);
        }
    }

    private class Dilate implements Function<Mat, Mat> {

        private Mat mKernel;
        private Mat mTempMat;

        public Dilate(int intensity) {
            setIntensity(intensity);
            this.mTempMat = new Mat();
        }

        @Override
        public Mat apply(Mat mat) {
            Imgproc.dilate(mat, mTempMat, mKernel);
            return mTempMat;
        }

        public void setIntensity(int intensity) {
            mKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(intensity, intensity));
        }
    }

    private class Erode implements Function<Mat, Mat> {

        private Mat mKernel;
        private Mat mTempMat;

        public Erode(int intensity) {
            setIntensity(intensity);
            this.mTempMat = new Mat();
        }

        @Override
        public Mat apply(Mat mat) {
            Imgproc.erode(mat, mTempMat, mKernel);
            return mTempMat;
        }

        public void setIntensity(int intensity) {
            mKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(intensity, intensity));
        }
    }

    private class Threshold implements Function<Mat, Mat> {

        private Scalar mMinColor;
        private Scalar mMaxColor;
        private Mat mTempMat;

        public Threshold(Scalar minColor, Scalar maxColor) {
            setColors(minColor, maxColor);
            this.mTempMat = new Mat();
        }

        @Override
        public Mat apply(Mat mat) {
            Core.inRange(mat, mMinColor, mMaxColor, mTempMat);
            return mTempMat;
        }

        public void setColors(Scalar minColor, Scalar maxColor) {
            mMinColor = minColor;
            mMaxColor = maxColor;
        }
    }

    private class AreaFilter implements Function<List<MatOfPoint>, List<MatOfPoint>> {

        private Range mRange;

        public AreaFilter(double min, double max) {
            setRange(min, max);
        }

        public void setRange(double min, double max) {
            mRange = new Range(min, max);
        }

        @Override
        public List<MatOfPoint> apply(List<MatOfPoint> candidates) {
            return candidates.stream().filter(contour -> mRange.contains(Imgproc.contourArea(contour))).collect(Collectors.toList());
        }
    }

    private class AspectRatio implements Function<List<MatOfPoint>, List<MatOfPoint>> {

        private Range mRange;

        public AspectRatio(double min, double max) {
            setRange(min, max);
        }

        public void setRange(double min, double max) {
            mRange = new Range(min, max);
        }

        @Override
        public List<MatOfPoint> apply(List<MatOfPoint> candidates) {
            return candidates.stream().filter(contour -> mRange.contains((float) Imgproc.boundingRect(contour).width / Imgproc.boundingRect(contour).height)).collect(Collectors.toList());
        }
    }

    private class Extent implements Function<List<MatOfPoint>, List<MatOfPoint>> {

        private Range mRange;

        public Extent(double min, double max) {
            setRange(min, max);
        }

        public void setRange(double min, double max) {
            mRange = new Range(min, max);
        }

        @Override
        public List<MatOfPoint> apply(List<MatOfPoint> candidates) {
            return candidates.stream().filter(contour -> mRange.contains(Imgproc.contourArea(contour) / Imgproc.boundingRect(contour).area())).collect(Collectors.toList());
        }
    }

    private class Largest implements Function<List<MatOfPoint>, List<MatOfPoint>> {

        private int mAmount;

        public Largest(int amount) {
            setAmount(amount);
        }

        public void setAmount(int amount) {
            mAmount = amount + 1;
        }

        @Override
        public List<MatOfPoint> apply(List<MatOfPoint> candidates) {
            PriorityQueue<Double> largest = new PriorityQueue<>(Collections.reverseOrder());
            List<MatOfPoint> l = new ArrayList<>();

            for (var candidate : candidates) {
                largest.add(Imgproc.contourArea(candidate));
            }

            Double smallest = -1.0;
            for (int i = 0; i < mAmount && !largest.isEmpty(); i++) {
                smallest = largest.remove();
            }

            for (int i = 0; i < candidates.size(); i++) {
                if (Imgproc.contourArea(candidates.get(i)) > smallest) {
                    l.add(candidates.get(i));
                }
            }
            return l;
        }
    }

    private class Percent implements Function<List<MatOfPoint>, List<MatOfPoint>> {

        private Range mRange;

        public Percent(double min, double max) {
            setRange(min, max);
        }

        public void setRange(double min, double max) {
            mRange = new Range(min, max);
        }

        @Override
        public List<MatOfPoint> apply(List<MatOfPoint> candidates) {
            return candidates.stream().filter(matOfPoint -> mRange.contains(Imgproc.contourArea(matOfPoint) / mImageSize)).collect(Collectors.toList());
        }
    }

    private class Perimeter implements Function<List<MatOfPoint>, List<MatOfPoint>> {

        private Range mRange;

        public Perimeter(double min, double max) {
            setRange(min, max);
        }

        public void setRange(double min, double max) {
            mRange = new Range(min, max);
        }

        @Override
        public List<MatOfPoint> apply(List<MatOfPoint> candidates) {
            MatOfPoint2f storage = new MatOfPoint2f();
            return candidates.stream().filter(contour -> {
                contour.convertTo(storage, CvType.CV_32FC2);
                return mRange.contains(Imgproc.arcLength(storage, true));
            }).collect(Collectors.toList());
        }
    }

    private class Solidity implements Function<List<MatOfPoint>, List<MatOfPoint>> {

        private Range mRange;

        public Solidity(double min, double max) {
            setRange(min, max);
        }

        public void setRange(double min, double max) {
            mRange = new Range(min, max);
        }

        @Override
        public List<MatOfPoint> apply(List<MatOfPoint> candidates) {
            return candidates.stream().filter(contour -> {
                MatOfInt hull = new MatOfInt();
                Imgproc.convexHull(contour, hull);
                Point[] candidateArray = contour.toArray();
                int[] hullArray = hull.toArray();
                MatOfPoint hullMat = new MatOfPoint();

                for (int value : hullArray) {
                    hullMat.push_back(new MatOfPoint(new Point(candidateArray[value].x, candidateArray[value].y)));
                }

                double solidity = Imgproc.contourArea(contour) / Imgproc.contourArea(hullMat);

                return mRange.contains(solidity);
            }).collect(Collectors.toList());
        }
    }

    private class findContours implements Function<Mat, List<MatOfPoint>> {

        @Override
        public List<MatOfPoint> apply(Mat mat) {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            return contours;
        }
    }

    public String getOrders() {
        return generateOrder(mOrders);
    }

    public void setOrders(String orders) {
        if (!verifyOrder(orders)) {
            throw new IllegalArgumentException("Order does not work");
        }
        this.mOrders = orders.split(",");
    }

    public List<MatOfPoint> parseMat(Mat input) {
        List<MatOfPoint> contours = null;
        Mat inputCopy = new Mat();
        input.copyTo(inputCopy);

        if (mOrders == null) {
            throw new NullPointerException("No orders");
        }

        for (var order : mOrders) {

        }

        return contours;
    }

    private boolean verifyOrder(String orders) {
        var processingImage = true;
        var parsedOrders = orders.split(",");

        // TODO: Make this look better
        for (var order : parsedOrders) {
            var type = order.split(" ")[0];
            if (type.equals("E")) {
                continue;
            }
            if (processingImage && type.equals("I")) {
            } else if (processingImage && type.equals("T")) {
                processingImage = false;
            } else if (!processingImage && type.equals("C")) {
            } else {
                return false;
            }
        }
        return true;
    }
}
