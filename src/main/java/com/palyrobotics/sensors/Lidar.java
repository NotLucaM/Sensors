package com.palyrobotics.sensors;

import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.esotericsoftware.kryonet.Server;
import com.fazecast.jSerialComm.SerialPort;

import java.io.IOException;

/*

 */

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
        server.getKryo().register(float[].class);
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

    public void verifyPort() {
        if (port != null) {
            return;
        }

        SerialPort[] ports = SerialPort.getCommPorts();
        for (var p : ports) {
            if (p.getSystemPortName().equals(portSystemName)) {
                // After a lot of testing these are the settings that work
                port = p;
                port.clearDTR();
                port.setRTS();
                port.setBaudRate(128000);
                port.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
                port.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, TIMEOUT, TIMEOUT); // TODO: Find out if timeout is in secs, and what timeout method to use                //port.clearDTR();
                port.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
                port.openPort();
                return;
            }
        }
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
        readHeader();
        while (port.isOpen()) {
            getDistance();
        }
    }

    public void readHeader() { // When you first write the command to start the motor, it will send a lot of redundant text first
        var stream = port.getInputStream();

        while (true) {
            try {
                int header = stream.read();
                if (header != 0xA5) continue;
                header = stream.read();
                if (header != 0x5A) continue;
                int length = stream.read();
                for (int i = 0; i < length - 1; i++) {
                    stream.read();
                }
                return;
            } catch (IOException ex) {
                return;
            }
        }
    }

    public void getDistance() { // See, I do sometimes write comments in my code :)
        var stream = port.getInputStream();

        try {
            while (true) { // Makes sure the header is the first thing that is read
                int header = (0xff & stream.read());
                int h2 = stream.read() & 0xFF;

                header = (h2 << 8) + header;
                if (header == 0x55AA) {
                    break;
                }
            }

            // Packets follow this order: package type (1B), sample quantity (1B), starting angle (2B), ending angle (2B), check sum (2B),
            // for the next sample quantity, there are 2B packets which represent the distance. The distances are equal angle measurements apart
            // In 2 byte numbers, the first byte is always the least significant byte and the second byte is the most significant
            int packageType = stream.read();
            int sampleQuantity = stream.read();
            float startingAngle = stream.read();
            startingAngle = (startingAngle + (stream.read() << 8)) / 128;
            float endAngle = stream.read();
            endAngle = (endAngle + (stream.read() << 8)) / 128;
            // Increase the end angle by 360 when it is smaller than the starting angle to not have negative delta
            if (endAngle < startingAngle) {
                endAngle += 360;
            }
            // Each distance will be stepAngle apart starting from startingAngle
            float stepAngle = (endAngle - startingAngle) / (float) sampleQuantity;
            int checkCode = stream.read();
            checkCode = checkCode << 8 + stream.read(); // TODO: actually use this

            // Read and send the distances
            for (int i = 0; i < sampleQuantity; i++) {
                int lsb = stream.read(); // TODO: make this nice
                int msb = stream.read();

                float angle = startingAngle + stepAngle * i;
                float distance = ((msb << 8) + lsb) / 4f;

                server.sendToAllTCP(new float[]{angle, distance}); // TODO: determine if TCP or UDP is better in this scenario
            }
        } catch (IOException ex) {
            System.err.println("Port not working correctly");
            ex.printStackTrace();
        }
    }
}
