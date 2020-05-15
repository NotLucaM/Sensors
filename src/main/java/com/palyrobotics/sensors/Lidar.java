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

    private final String description;
    private final Server server;
    private final int tcpPort;
    private SerialPort serialPort;
    private boolean running;

    public Lidar(String description, int tcpPort)  {
        this.description = description;
        this.running = false;
        this.server = new Server();
        this.tcpPort = tcpPort;

        openPort();
        setUpServer();
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

    public void openPort() {
        if (serialPort != null) {
            return;
        }

        SerialPort[] ports = SerialPort.getCommPorts();
        for (var p : ports) {
            if (p.getSystemPortName().contains(description) || p.getDescriptivePortName().contains(description)) {
                // After a lot of testing these are the settings that work
                serialPort = p;
                serialPort.clearDTR();
                serialPort.setRTS();
                serialPort.setBaudRate(128000);
                serialPort.setFlowControl(SerialPort.FLOW_CONTROL_DISABLED);
                serialPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, TIMEOUT, TIMEOUT); // TODO: Find out if timeout is in secs, and what timeout method to use                //port.clearDTR();
                serialPort.openPort();
//                try {
//                    Thread.sleep(1000);
//                } catch (InterruptedException e) {
//                    e.printStackTrace();
//                }
                return;
            }
        }
    }

    @Override
    public void init() {
        if (serialPort != null && !running) {
            new Thread(this).start();
        }
    }

    @Override
    public void terminate() {
        serialPort.writeBytes(new byte[] { (byte) 0xA5, (byte) 0x65 }, 2, 0); // Stops the motor and the scanner
        serialPort.closePort(); // Should generate and error for the thread and close it
    }

    @Override
    public String getName() {
        return "TimeOfFlightSensor";
    }

    @Override
    public void run() {
        running = true;

        openPort();

        serialPort.writeBytes(new byte[] {(byte) 0xA5, (byte) 0x60 }, 2, 0); // Starts the motor and the scanner
        readHeader();
        while (serialPort.isOpen()) {
            getDistance();
        }
    }

    public void readHeader() { // When you first write the command to start the motor, it will send a lot of redundant text first
        var stream = serialPort.getInputStream();

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
        var stream = serialPort.getInputStream();

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

                var msg = new float[]{angle > 360 ? angle - 360 : angle, distance};  // the lidar sometimes gives angles above 360, idk why?
                var connectionList = server.getConnections();
                for (var connection: connectionList) { // I know sendTCPToAll exists, but it gave bugs so...
                    try {
                        server.sendToTCP(connection.getID(), msg); // TODO: determine if TCP or UDP is better in this scenario
                    } catch (Throwable t) {
                        System.err.println("Closing " + connection.getID());
                        connection.close();
                    }
                }
            }
        } catch (IOException ex) {
            System.err.println("Port not working correctly");
            ex.printStackTrace();
        }
    }
}
