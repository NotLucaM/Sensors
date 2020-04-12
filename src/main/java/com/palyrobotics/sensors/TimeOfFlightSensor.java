package com.palyrobotics.sensors;

import com.esotericsoftware.kryo.io.Input;
import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.esotericsoftware.kryonet.Server;
import com.fazecast.jSerialComm.SerialPort;

import java.io.IOException;
import java.io.InputStream;

public class TimeOfFlightSensor implements Sensor {

    static final int TIMEOUT = 2000;

    private final String portSystemName;
    private SerialPort port;
    private boolean running;
    private Server server;
    private final int tcpPort;

    public TimeOfFlightSensor(String portSystemName, int tcpPort)  {
        this.portSystemName = portSystemName;
        this.running = false;
        this.server = new Server();
        this.tcpPort = tcpPort;

        verifyPort();
    }

    private void setUpServer() { // TODO: maybe make Sensor an abstract class with this as it is similar to KumquatVision
        server.start();
        server.addListener(new Listener() {
            @Override
            public void connected(Connection connection) {
                System.out.println("Connected");
            }

            @Override
            public void disconnected(Connection connection) {
                System.out.println("Disconnected");
            }
        });
        try {
            server.bind(tcpPort);
        } catch (IOException connectException) {
            connectException.printStackTrace();
        }
    }

    public boolean verifyPort() {
        if (port != null) {
            return true;
        }

        SerialPort[] ports = SerialPort.getCommPorts();
        for (var p : ports) {
            if (p.getSystemPortName().equals(portSystemName)) {
                port = p;
                port.setBaudRate(115200);
                port.setComPortTimeouts(SerialPort.TIMEOUT_NONBLOCKING, TIMEOUT, TIMEOUT); // TODO: Find out if timeout is in secs, and what timeout method to use
                return true;
            }
        }
        return false;
    }

    @Override
    public void init() {
        if (port != null && !running) {
            new Thread(this).start();
        }
    }

    @Override
    public void terminate() {
        port.closePort(); // Should generate and error for the thread and close it
    }

    @Override
    public String getName() {
        return "TimeOfFlightSensor";
    }

    @Override
    public void run() {
        running = true;

        while (true) {
            int distance = getDistance();
            server.sendToAllTCP(distance); // TODO: determine if TCP or UDP is better in this scenario
        }
    }

    public int getDistance() {
        var stream = port.getInputStream();

        while (true) { // I believe the port or the InputStream will throw an error if it does not work and the loop will be broken because of that
            try {
                // Each 9 byte data package has 2 headers (both 0x59) followed by 2 bytes representing the distance
                int header = stream.read();
                while (header != 0x59) { // Continue reading until the package header is read
                    header = stream.read();
                }
                header = stream.read();
                if (header != 0x59) { // Make sure the next byte is also a header
                    continue;
                }

                // Finds the distance from the next 2 bytes
                int distance = stream.read();
                distance = distance + 256 * stream.read();
                return distance;

            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
