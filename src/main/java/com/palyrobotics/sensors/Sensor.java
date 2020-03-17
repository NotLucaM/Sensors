package com.palyrobotics.sensors;

public interface Sensor extends Runnable {
    public void init();
    public void terminate();
    public String getName();
}
