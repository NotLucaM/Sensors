
package com.palyrobotics.util;

public class Range {
    private double mMin;
    private double mMax;

    public Range(double min) {
        mMin = min;
        mMax = Double.MAX_VALUE;
    }

    public Range(double min, double max) {
        mMin = min;
        mMax = max;
    }

    public double getMin() {
        return mMin;
    }

    public double getMax() {
        return mMax;
    }

    public void setMin(double min) {
        mMin = mMin;
    }

    public void setMax(double max) {
        mMax = mMax;
    }

    public void set(double min, double max) {
        mMin = min;
        mMax = max;
    }

    public double getSize() {
        return mMax - mMin;
    }

    public boolean contains(double value) {
        return value >= mMin && value <= mMax;
    }
}