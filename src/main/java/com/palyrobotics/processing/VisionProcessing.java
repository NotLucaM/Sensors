package com.palyrobotics.processing;

import com.esotericsoftware.minlog.Log;
import com.palyrobotics.sensors.KumquatVision;
import com.palyrobotics.util.Range;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;
import java.util.stream.Collectors;

public class VisionProcessing {

    private List<Order> mOrders;
    private static double mImageSize = KumquatVision.kCaptureHeight * KumquatVision.kCaptureWidth;

    public VisionProcessing() {
    }

    public VisionProcessing(List<Order> orders) {
        setOrders(orders);
    }

    public static List<Order> generateOrder(Order... orders) {
        StringBuilder builder = new StringBuilder();
        List<Order> finalOrder = new ArrayList<>();

        for (var order : orders) {
            if (TransformMat.class.isAssignableFrom(order.getClass()) ||
                FilterContour.class.isAssignableFrom(order.getClass()) ||
                FindContour.class.isAssignableFrom(order.getClass())) {

                builder.append(order.toString());
                finalOrder.add(order);
            } else {
                throw new IllegalArgumentException();
            }
        }
        Log.info(builder.toString());
        return finalOrder;
    }

    public interface Order {
    }

    public interface TransformMat extends Order {
        public Mat apply(Mat object);
    }

    public interface FilterContour extends Order {
        public List<MatOfPoint> apply(List<MatOfPoint> object);
    }

    public interface FindContour extends Order {
        public List<MatOfPoint> apply(Mat object);
    }

    // TODO: use Jackson to store in json

    public static class Blur implements TransformMat {

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

        @Override
        public String toString() {
            return "Blur";
        }
    }

    public static class Dilate implements TransformMat {

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

        @Override
        public String toString() {
            return "Dilate";
        }
    }

    public static class Erode implements TransformMat {

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

        @Override
        public String toString() {
            return "Erode";
        }
    }

    public static class Threshold implements TransformMat {

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

        @Override
        public String toString() {
            return "Threshold";
        }
    }

    public static class Area implements FilterContour {

        private Range mRange;

        public Area(double min, double max) {
            setRange(min, max);
        }

        public void setRange(double min, double max) {
            mRange = new Range(min, max);
        }

        @Override
        public List<MatOfPoint> apply(List<MatOfPoint> candidates) {
            return candidates.stream().filter(contour -> mRange.contains(Imgproc.contourArea(contour))).collect(Collectors.toList());
        }

        @Override
        public String toString() {
            return "Area";
        }
    }

    public static class AspectRatio implements FilterContour {

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

        @Override
        public String toString() {
            return "Aspect Ratio";
        }
    }

    public static class Extent implements FilterContour {

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

        @Override
        public String toString() {
            return "Extent";
        }
    }

    public static class Largest implements FilterContour {

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

        @Override
        public String toString() {
            return "Largest";
        }
    }

    public static class Percent implements FilterContour {

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

        @Override
        public String toString() {
            return "Percent";
        }
    }

    public static class Perimeter implements FilterContour {

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

        @Override
        public String toString() {
            return "Perimeter";
        }
    }

    public static class Solidity implements FilterContour {

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

        @Override
        public String toString() {
            return "Solidity";
        }
    }

    public static class FindContours implements FindContour {

        @Override
        public List<MatOfPoint> apply(Mat mat) {
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mat, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            return contours;
        }

        @Override
        public String toString() {
            return "Find Contours";
        }
    }

    public void setOrders(List<Order> orders) {
        if (!verifyOrder(orders)) {
            StringBuilder orderName = new StringBuilder();
            orders.stream().map(order -> order.toString()).forEach(orderName::append);
            throw new IllegalArgumentException(String.format("Order does not work: %s", orderName.toString()));
        }
        mOrders = orders;
    }

    public List<MatOfPoint> apply(Mat input) {
        List<MatOfPoint> contours = null;
        Mat inputCopy = new Mat();
        input.copyTo(inputCopy);

        for (var order : mOrders) {
            if (TransformMat.class.isAssignableFrom(order.getClass())) {
                inputCopy = ((TransformMat) order).apply(inputCopy);
            } else if (FilterContour.class.isAssignableFrom(order.getClass())) {
                contours = ((FilterContour) order).apply(contours);
            } else if (FindContour.class.isAssignableFrom(order.getClass())) {
                contours = ((FindContour) order).apply(inputCopy);
            }
        }

        return contours;
    }

    private static boolean verifyOrder(List<Order> orders) {
        boolean threshold = false;
        for (var order : orders) {
            if (order.getClass().equals(Threshold.class)) {
                threshold = true;
            }
            if (FindContour.class.isAssignableFrom(order.getClass()) && !threshold) {
                return false;
            }
            if (FindContour.class.isAssignableFrom(order.getClass()) && threshold) {
                return true;
            }
        }
        return false;
    }
}
