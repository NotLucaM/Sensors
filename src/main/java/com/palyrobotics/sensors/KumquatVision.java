package com.palyrobotics.sensors;

import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.esotericsoftware.kryonet.Server;
import com.palyrobotics.util.Address;
import com.palyrobotics.processing.VisionProcessing;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import java.io.IOException;
import java.util.logging.Logger;

public class KumquatVision implements Sensor {

    public static final int BUFFER_SIZE = 50000;
    private static final long SLEEP_MS = 100;

    private final Address mImageAddress;
    private final Address mDataAddress;
    private final Server mImageServer;
    private final Server mDataServer;
    private final VideoCapture mCapture;
    private final String mWindowName;
    private final VisionProcessing mProcesser;

    private final Mat mCaptureMat;
    private final MatOfByte mStreamMat;

    private boolean mShowImage;
    private boolean mThreadRunning;
    private int mCaptureFps = 60;
    public static int kCaptureWidth = 1024;
    public static int kCaptureHeight = 1024;
    private String mOrder;

    Logger l = Logger.getGlobal();

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    public KumquatVision(String address, Integer imageTcpPort, Integer imageUdpPort, Integer dataTcpPort, Integer dataUdpPort) {
        this(address, imageTcpPort, imageUdpPort, dataTcpPort, dataUdpPort, false, "");
    }

    public KumquatVision(String address, Integer imageTcpPort, Integer imageUdpPort, Integer dataTcpPort, Integer dataUdpPort, boolean showImage, String windowName) { // TODO: remove show image
        mImageAddress = new Address(address, imageTcpPort, imageUdpPort);
        mDataAddress = new Address(address, dataTcpPort, dataUdpPort);
        mWindowName = windowName;

        mCapture = new VideoCapture(0);
        mThreadRunning = false;
        mImageServer = new Server();
        mDataServer = new Server();

        mShowImage = showImage;
        mCaptureMat = new Mat();
        mStreamMat = new MatOfByte();
        mProcesser = new VisionProcessing();
    }

    private void setUpCapture() {
        mCapture.set(Videoio.CAP_PROP_FRAME_WIDTH, kCaptureWidth);
        mCapture.set(Videoio.CAP_PROP_FRAME_HEIGHT, kCaptureHeight);
        mCapture.set(Videoio.CAP_PROP_FPS, mCaptureFps);
    }

    public void setShowImage(boolean b) {
        mShowImage = b;
        maybeStart();
    }

    /**
     * Start a new thread, if not already running and needed:
     * - showImage is enabled
     */
    private synchronized void maybeStart() {
        if (!mThreadRunning) {
            mThreadRunning = true;
            new Thread(this).start();
        }
    }

    private synchronized void onStop() {
        mThreadRunning = false;
    }

    public void setOrder(String order) {
        mOrder = order;
        mProcesser.setOrders(order);
    }

    @Override
    public void run() {
        try {
            l.info("Starting capture thread");
            while (mCapture.isOpened()) {
                boolean hasClients = mImageServer.getConnections().length > 0;
                // If not showing image and not sending to clients - exit thread.
                if (!hasClients && !mShowImage) {
                    return;
                }
                if (readFrame()) {
                    sendFrameToConnectedClients();
                }
            }
        } finally {
            onStop();
        }
    }

    private void setUpServers() {
        setUpServer(mImageServer, byte[].class);
        setUpServer(mDataServer);
    }

    private void setUpServer(Server server, Class... types) {
        for (Class type : types) {
            server.getKryo().register(type);
        }
        server.start();
        server.addListener(new Listener() {
            @Override
            public void connected(Connection connection) {
                maybeStart();
                System.out.println("Connected");
            }

            @Override
            public void disconnected(Connection connection) {
                System.out.println("Disconnected");
            }
        });
        try {
            server.bind(mImageAddress.getTcpPort(), mImageAddress.getUdpPort());
        } catch (IOException connectException) {
            connectException.printStackTrace();
        }
    }

    private boolean readFrame() {
        if (mCapture.read(mCaptureMat)) {
            mProcesser.parseMat(mCaptureMat);
            if (mShowImage) {
                HighGui.imshow(mWindowName, mCaptureMat);
                HighGui.waitKey(1);
            }
            return Imgcodecs.imencode(".jpg", mCaptureMat, mStreamMat, new MatOfInt(Imgcodecs.IMWRITE_JPEG_QUALITY, 40));
        } else {
            System.err.println("Opened camera, but could not read from it.");
            return false;
        }
    }

    private void sendFrameToConnectedClients() {
        for (Connection connection : mImageServer.getConnections()) {
            if (connection.isConnected()) {
                final var bytes = mStreamMat.toArray();
                if (bytes.length < BUFFER_SIZE) {
                    connection.sendTCP(bytes);
                } else {
                        System.err.printf("Expected buffer size less than %d, got %d", BUFFER_SIZE, bytes.length);
                }
            }
        }
    }

    @Override
    public void init() {
        setUpServers();
        setUpCapture();
        maybeStart();
    }

    @Override
    public void terminate() {
        HighGui.destroyWindow(mWindowName);
        mCapture.release();
    }

    @Override
    public String getName() {
        return "KumquatVisionModule";
    }
}
