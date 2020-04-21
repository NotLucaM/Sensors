package com.palyrobotics.sensors;

import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.esotericsoftware.kryonet.Server;
import com.fazecast.jSerialComm.SerialPort;

import java.io.IOException;

public class Lidar implements Sensor {

    // TODO: make an abstract class for SerialSensors, it should have the baudrate and serial port as fields

    static final int TIMEOUT = 10000;

    private final String portSystemName;
    private SerialPort port;
    private boolean running;
    private Server server;
    private final int tcpPort;

    public Lidar(String portSystemName, int tcpPort)  {
        this.portSystemName = portSystemName;
        this.running = false;
        this.server = new Server();
        this.tcpPort = tcpPort;

        setUpServer();
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
                port.setBaudRate(128000);
                port.clearDTR();
                port.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
                port.openPort();
                port.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, TIMEOUT, TIMEOUT); // TODO: Find out if timeout is in secs, and what timeout method to use                //port.clearDTR();
                port.setRTS();
                port.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
                return true;
            }
        }
        return false;
    }

    @Override
    public void init() {
        verifyPort();

        if (port != null && !running) {
            new Thread(this).start();
        }
    }

    @Override
    public void terminate() {
        port.writeBytes(new byte[] { (byte) 0xA5, (byte) 0x65 }, 2, 0); // Stops the motor and the scanner
        port.closePort(); // Should generate and error for the thread and close it
    }

    @Override
    public String getName() {
        return "TimeOfFlightSensor";
    }

    @Override
    public void run() {
        running = true;

        port.writeBytes(new byte[] {(byte) 0xA5, (byte) 0x60 }, 2, 0); // Starts the motor and the scanner
//        readDistanceReply();
        while (port.isOpen()) {
            int distance = getDistance();
            port.writeBytes(new byte[] {(byte) 0xA5, (byte) 0x60 }, 2, 0); // Starts the motor and the scanner
            server.sendToAllTCP(distance); // TODO: determine if TCP or UDP is better in this scenario
        }
    }

//    public boolean readDistanceReply() {
//        var stream = port.getInputStream();
//
//        while (true) {
//            try {
//                int header = stream.read();
//                if (header != 0xA5) continue;
//                header = stream.read();
//                if (header != 0x5A) continue;
//                while (true) {
//                    System.out.println(Integer.toHexString(stream.read()));
//                }
//            } catch (IOException ex) {
//                return false;
//            }
//        }
//    }

    public int getDistance() { // See, I do sometimes write comments in my code :)
        var stream = port.getInputStream();

        while (true) { // I believe the port or the InputStream will throw an error if it does not work and the loop will be broken because of that
            try {
                // The header is made up of two bytes 0xA5 and 0x5A
                int header = stream.read();
                if (header != 0xA5) continue;
                header = stream.read();
                if (header != 0x5A) continue;
                System.out.println("READ HEADER");

                // The way the length is read is very messed up,
                int len = stream.read();
                len = len  + (byte) stream.read() << 8;
                len = len << 8 + (byte) stream.read();
                len = len << 8 + (byte) stream.read();
                int type = len & 0x3;
                len = len >> 2;

                int packageType = stream.read();
                int sampleQuantity = stream.read();
                int startingAngle = stream.read();
                startingAngle = startingAngle * 256 + stream.read();
                int endAngle = stream.read();
                endAngle = endAngle * 256 + stream.read();
                int checkCode = stream.read();
                checkCode = checkCode * 256 + stream.read();
                int samplingData = stream.read();
                samplingData = samplingData * 256 + stream.read();

                return startingAngle;

            } catch (IOException e) {
                //e.printStackTrace();
                break;
            }
        }

        return -1;
    }
}
