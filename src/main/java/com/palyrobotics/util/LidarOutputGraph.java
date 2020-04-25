package com.palyrobotics.util;

import com.esotericsoftware.kryonet.Client;
import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

import java.io.IOException;
import java.util.Arrays;

public class LidarOutputGraph extends Application {

    private final String host = "";
    private final int tcpPort = 5807;
    private Client client;
    private double pointRadius = 10;
    private long lastClear;

    private GraphicsContext gc;

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) throws Exception {
        primaryStage.setTitle("Lidar Output");
        Group root = new Group();
        Canvas canvas = new Canvas(1500, 1500);
        gc = canvas.getGraphicsContext2D();
        root.getChildren().add(canvas);
        primaryStage.setScene(new Scene(root));
        primaryStage.show();

        setUpClient();
    }

    private void setUpClient() throws IOException {
        Client client = new Client();
        client.getKryo().register(float[].class);
        client.addListener(new Listener() {
            @Override
            public void connected(Connection connection) {
                System.out.println("Connected");
            }

            @Override
            public void received(Connection connection, Object object) {
                if (object instanceof float[]) {
                    drawShapes((float[]) object);
                }
            }
        });
        new Thread(client).start();

        client.connect(4000, host, tcpPort);
    }

    private void drawShapes(float[] polar) {
        float[] cartesian = getCartesian(polar);

        gc.setFill(Color.RED);
        gc.setStroke(Color.BLACK);

        if (System.currentTimeMillis() - lastClear >= 1000) {
            lastClear = System.currentTimeMillis();
            gc.clearRect(0,0,1500,1500);
        }

        gc.fillOval((cartesian[0] - pointRadius) / 3 + 750,
                (cartesian[1] - pointRadius) / 3 + 750,
                pointRadius, pointRadius);
    }

    public float[] getCartesian(float[] polar) {
        return new float[]{(float) (polar[1] * Math.cos(Math.toRadians(polar[0]))),
                (float) (polar[1] * Math.sin(Math.toRadians(polar[0])))};
    }
}
