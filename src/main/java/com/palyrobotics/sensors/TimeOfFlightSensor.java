package com.palyrobotics.sensors;

import com.fazecast.jSerialComm.SerialPort;

import java.io.IOException;
import java.io.InputStream;

public class TimeOfFlightSensor implements Sensor {

    static final  int TIMEOUT = 2000;

    private final String portSystemName;
    private SerialPort port;

    public TimeOfFlightSensor(String portSystemName)  {
        this.portSystemName = portSystemName;
        verifyPort();
    }

    public boolean verifyPort() {
        SerialPort[] ports = SerialPort.getCommPorts();
        for (var p : ports) {
            if (p.getSystemPortName().equals(portSystemName)) {
                port = p;
                return true;
            }
        }
        return false;
    }

    @Override
    public void init() {
        port.openPort();
    }

    @Override
    public void terminate() {
        port.closePort();
    }

    @Override
    public String getName() {
        return "TimeOfFlightSensor";
    }

    @Override
    public void run() {

    }

    public void searchForPorts() {
        SerialPort[] ports = SerialPort.getCommPorts();
        for(var p : ports) {
            System.out.println(p.getDescriptivePortName() + " " + p.getSystemPortName() + " " + p.getBaudRate());

            p.setBaudRate(115200);
            p.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, 1000, 1000);
            p.openPort();
            InputStream br = p.getInputStream();
            while (true) {
                try {
                    int n1 = br.read();
                    while (n1 != 0x59) {
                        n1 = br.read();
                    }
                    n1 = br.read();
                    if (n1 != 0x59) {
                        continue;
                    }
                    int n2 = br.read();
                    n2 = n2 + 256 * br.read();
                    System.out.println(n2);
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
