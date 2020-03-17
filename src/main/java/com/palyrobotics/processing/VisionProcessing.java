package com.palyrobotics.processing;

import com.palyrobotics.util.ColorConstants;
import com.palyrobotics.sensors.KumquatVisionModule;
import com.palyrobotics.util.Range;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;
import java.util.stream.Collectors;

public class VisionProcessing {

    private static final String kErode = "erode";
    private static final String kDilate = "dilate";
    private static final String kBlur = "blur";
    private static final String kThreshold = "threshold";
    private static final String kArea = "area";
    private static final String kPercent = "percent";
    private static final String kExtent = "extent";
    private static final String kSolidity = "solidity";
    private static final String kAspect = "aspect";
    private static final String kPerimeter = "perimeter";
    private static final String kSort = "sort";
    private static final String kLargest = "largest";

    private String[] mOrders;
    private Mat mTempMat = new Mat();
    private Mat mDilateKernel;
    private Mat mErodeKernel;
    private Size mBlurSize;
    private Scalar mMinColor;
    private Scalar mMaxColor;
    private Range mAreaRange;
    private Range mPercentRange;
    private Range mExtentRange;
    private Range mSolidityRange;
    private Range mAspectRange;
    private Range mPerimeterRange;
    private double mImageSize = KumquatVisionModule.kCaptureHeight * KumquatVisionModule.kCaptureWidth;

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

    // TODO: use Jackson to do this better
    public static String erode(int intensity) {
        return String.format("I %s %d,", kErode, intensity);
    }

    public static String dilate(int intensity) {
        return String.format("I %s %d,", kDilate, intensity);
    }

    public static String blur(int intensity) {
        return String.format("I %s %d,", kBlur, intensity);
    }

    public static String threshold(Scalar min, Scalar max) {
        return String.format("T %s %f %f %f %f %f %f,", kErode, min.val[0], min.val[1], min.val[2], max.val[0], max.val[1], max.val[2]);
    }

    public static String areaFilter(double min, double max) {
        return String.format("C %s %f %f,", kArea, min, max);
    }

    public static String aspectRatio(double min, double max) {
        return String.format("C %s %f %f,", kAspect, min, max);

    }

    public static String extent(double min, double max) {
        return String.format("C %s %f %f,", kExtent, min, max);

    }

    public static String largest(long amount) {
        return String.format("C %s %d,", kLargest, amount);
    }

    public static String percent(double min, double max) {
        return String.format("C %s %f %f,", kPercent, min, max);
    }

    public static String perimeter(double min, double max) {
        return String.format("C %s %f %,f", kPerimeter, min, max);
    }

    public static String solidity(double min, double max) {
        return String.format("C %s %f %f,", kSolidity, min, max);
    }

    public static String display() {
        return "E display,";
    }

    public static String showContour() {
        return "E contour,";
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
            var parsed = order.split(" ");
            if (parsed[0].equals("T")) {
                inputCopy = threshold(inputCopy, Double.parseDouble(parsed[2]), Double.parseDouble(parsed[3]), Double.parseDouble(parsed[4]),
                        Double.parseDouble(parsed[5]), Double.parseDouble(parsed[6]), Double.parseDouble(parsed[7]));
                contours = findContours(inputCopy);
                continue;
            }
            if (parsed[0].equals("E")) {
                switch (parsed[1]) {
                    case "display":
                        Imgproc.ellipse(input, new Point(0, 0), new Size(100, 100), 50, 0, 50, new Scalar(0, 0, 0));
                        break;
                    case "copy":
                        inputCopy.copyTo(input);
                        break;
                    case "contour":
                        for (int i = 0; i < contours.size(); i++) {
                            Imgproc.drawContours(input, contours, i, ColorConstants.kRedUpperBoundHSV, 25); // TODO: when the image is greyscale display this in color
                        }
                        break;
                }

                continue;
            }
            switch (parsed[1]) {
                case kErode:
                    inputCopy = erode(inputCopy, Integer.parseInt(parsed[2]));
                    break;
                case kBlur:
                    inputCopy = blur(inputCopy, Integer.parseInt(parsed[2]));
                    break;
                case kDilate:
                    inputCopy = dilate(inputCopy, Integer.parseInt(parsed[2]));
                    break;
                case kArea:
                    contours = areaFilter(contours, Double.parseDouble(parsed[2]), Double.parseDouble(parsed[3]));
                    break;
                case kAspect:
                    contours = aspectRatio(contours, Double.parseDouble(parsed[2]), Double.parseDouble(parsed[3]));
                    break;
                case kExtent:
                    contours = extent(contours, Double.parseDouble(parsed[2]), Double.parseDouble(parsed[3]));
                    break;
                case kLargest:
                    contours = largest(contours, Long.parseLong(parsed[2]));
                    break;
                case kPercent:
                    contours = percent(contours, Double.parseDouble(parsed[2]), Double.parseDouble(parsed[3]));
                    break;
                case kPerimeter:
                    contours = perimeter(contours, Double.parseDouble(parsed[2]), Double.parseDouble(parsed[3]));
                    break;
                case kSolidity:
                    contours = solidity(contours, Double.parseDouble(parsed[2]), Double.parseDouble(parsed[3]));
                    break;
                case kSort:
                    break;
                default:
                    throw new IllegalArgumentException("Unknown order");
            }
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

    private Mat blur(Mat object, int size) {
        if (mBlurSize == null || mBlurSize.height != size) {
            mBlurSize = new Size(size, size);
        }
        Imgproc.blur(object, mTempMat, mBlurSize);
        return mTempMat;
    }

    private Mat dilate(Mat object, int size) {
        if (mDilateKernel == null || mDilateKernel.size().height != size) {
            mDilateKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(size, size));
        }
        Imgproc.dilate(object, mTempMat, mDilateKernel);
        return mTempMat;
    }

    private Mat erode(Mat object, int size) {
        if (mErodeKernel == null || mErodeKernel.size().height != size) {
            mErodeKernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(size, size));
        }
        Imgproc.erode(object, mTempMat, mErodeKernel);
        return mTempMat;
    }

    private Mat threshold(Mat object, double min1, double min2, double min3, double max1, double max2, double max3) {
        if (mMinColor == null || mMinColor.val[0] != min1 || mMinColor.val[2] != min2 || mMinColor.val[3] != min3) {
            mMinColor = new Scalar(min1, min2, min3);
        }
        if (mMaxColor == null || mMaxColor.val[0] != max1 || mMaxColor.val[2] != max2 || mMaxColor.val[3] != max3) {
            mMaxColor = new Scalar(max1, max2, max3);
        }
        Core.inRange(object, mMinColor, mMaxColor, mTempMat);
        return mTempMat;
    }

    private List<MatOfPoint> areaFilter(List<MatOfPoint> candidates, double min, double max) {
        mAreaRange.set(min, max);
        return candidates.stream().filter(contour -> mAreaRange.contains(Imgproc.contourArea(contour))).collect(Collectors.toList());
    }

    public List<MatOfPoint> aspectRatio(List<MatOfPoint> candidates, double min, double max) {
        mAspectRange.set(min, max);
        return candidates.stream().filter(contour -> mAspectRange.contains((float) Imgproc.boundingRect(contour).width / Imgproc.boundingRect(contour).height)).collect(Collectors.toList());
    }

    public List<MatOfPoint> extent(List<MatOfPoint> candidates, double min, double max) {
        mExtentRange.set(min, max);
        return candidates.stream().filter(contour -> mExtentRange.contains(Imgproc.contourArea(contour) / Imgproc.boundingRect(contour).area())).collect(Collectors.toList());
    }

    public List<MatOfPoint> largest(List<MatOfPoint> candidates, long amount) { // TODO: optimize this
        amount++; // idk why but it works
        PriorityQueue<Double> largest = new PriorityQueue<>(Collections.reverseOrder());
        List<MatOfPoint> l = new ArrayList<>();

        for (var candidate : candidates) {
            largest.add(Imgproc.contourArea(candidate));
        }

        Double smallest = -1.0;
        for (int i = 0; i < amount && !largest.isEmpty(); i++) {
            smallest = largest.remove();
        }

        for (int i = 0; i < candidates.size(); i++) {
            if (Imgproc.contourArea(candidates.get(i)) > smallest) {
                l.add(candidates.get(i));
            }
        }
        return l;
    }

    public List<MatOfPoint> percent(List<MatOfPoint> candidates, double min, double max) {
        mPercentRange.set(min, max);
        return candidates.stream().filter(matOfPoint -> mPercentRange.contains(Imgproc.contourArea(matOfPoint) / mImageSize)).collect(Collectors.toList());
    }

    public List<MatOfPoint> perimeter(List<MatOfPoint> candidates, double min, double max) {
        mPercentRange.set(min, max);
        MatOfPoint2f storage = new MatOfPoint2f();
        return candidates.stream().filter(contour -> {
            contour.convertTo(storage, CvType.CV_32FC2);
            return mPerimeterRange.contains(Imgproc.arcLength(storage, true));
        }).collect(Collectors.toList());
    }

    public List<MatOfPoint> solidity(List<MatOfPoint> candidates, double min, double max) { // TODO: Optimize and make it look better
        mPercentRange.set(min, max);
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

            return mSolidityRange.contains(solidity);
        }).collect(Collectors.toList());
    }

    private List<MatOfPoint> findContours(Mat src) {
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(src, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        return contours;
    }
}
