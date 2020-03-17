package com.palyrobotics.sensors;

import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import com.esotericsoftware.kryonet.Server;
import com.palyrobotics.processing.VisionProcessing;
import com.palyrobotics.util.Address;
import com.palyrobotics.util.ColorConstants;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.logging.Logger;

public class KumquatVision implements Sensor {

    public static final int BUFFER_SIZE = 50000;
    private static final long SLEEP_MS = 100;
    public static int kCaptureWidth = 1024;
    public static int kCaptureHeight = 1024;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    private final Address mImageAddress;
    private final Address mDataAddress;
    private final Server mImageServer;
    private final Server mDataServer;
    private final VideoCapture mCapture;
    private final String mWindowName;
    private final Set<VisionProcessing> mProcessers;
    private final Set<VisionProcessing> mRunningProcesses;
    private final Mat mCaptureMat;
    private final MatOfByte mStreamMat;
    Logger l = Logger.getGlobal();
    private boolean mShowImage;
    private boolean mDrawContours;
    private boolean mThreadRunning;
    private int mCaptureFps = 60;
    private Double mTx = null;
    private Double mTy = null;
    private Double mSkew = null;
    private List<MatOfPoint> mContour;
    private String mOrder;

    public KumquatVision(String address, Integer imageTcpPort, Integer imageUdpPort, Integer dataTcpPort, Integer dataUdpPort) {
        this(address, imageTcpPort, imageUdpPort, dataTcpPort, dataUdpPort, false, "");
    }

    public KumquatVision(String address, Integer imageTcpPort, Integer imageUdpPort, Integer dataTcpPort, Integer dataUdpPort, boolean showImage, String windowName) {
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
        mProcessers = new HashSet<>();
        mRunningProcesses = new HashSet<>();
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

    public void addPipeline(String pipeline) {
        addPipeline(new VisionProcessing(pipeline));
    }

    public void addActive(String pipeline) {
        addActive(new VisionProcessing(pipeline));
    }

    public void addPipeline(VisionProcessing pipeline) {
        mProcessers.add(pipeline);
    }

    public void addActive(VisionProcessing pipeline) {
        mProcessers.add(pipeline);
        mRunningProcesses.add(pipeline);
    }

    public void removeActive(VisionProcessing pipeline) {
        if (!mRunningProcesses.contains(pipeline)) {
            throw new IllegalArgumentException("Pipeline is not running");
        }
        mRunningProcesses.remove(pipeline);
    }

    @Override
    public void run() {
        try {
            l.info("Starting capture thread");
            while (mCapture.isOpened()) {
                boolean hasImageClient = mImageServer.getConnections().length > 0;
                boolean hasDataClient = mDataServer.getConnections().length > 0;
                // If not showing image and not sending to clients - exit thread.
                if (!(hasDataClient || hasImageClient) && !mShowImage) {
                    return;
                }
                if (readEditFrame()) {
                    if (hasImageClient) {
                        sendFrameToConnectedClients();
                    } else {
                        sendDataToConnectedClients();
                    }
                }
            }
        } finally {
            onStop();
        }
    }

    private void setUpServers() {
        setUpServer(mImageServer, byte[].class);
        setUpServer(mDataServer, MatOfPoint.class, List.class, Double.class); // TODO: implement the consumer to change the current pipeline efficiently (for control center)
    }

    private void setUpServer(Server server, BiConsumer<Connection, Object> received, Class... types) {
        setUpServer(server, types);
        server.addListener(new Listener() {
            @Override
            public void received(Connection connection, Object o) {
                received.accept(connection, o);
            }
        });
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

    private boolean readEditFrame() {
        if (mCapture.read(mCaptureMat)) {
            List<MatOfPoint> contours = new ArrayList<>();
            for (VisionProcessing process : mProcessers) {
                contours.addAll(process.parseMat(mCaptureMat));
            }
            if (mDrawContours) {
                for (int i = 0; i < contours.size(); i++) {
                    Imgproc.drawContours(mCaptureMat, contours, i, ColorConstants.kRedUpperBoundHSV, 25); // TODO: when the image is greyscale display this in color
                }
            }
            if (mShowImage) { // TODO: remove this and add show it through the network
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

    private void sendDataToConnectedClients() {
        for (Connection connection : mImageServer.getConnections()) {
            if (connection.isConnected()) {
                connection.sendTCP(List.of(mTx, mTy, mSkew, mContour));
            }
        }
    }

    public void setDrawContours(boolean drawContours) {
        this.mDrawContours = drawContours;
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
