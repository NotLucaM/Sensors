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

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) throws Exception {
        setUpClient();

        primaryStage.setTitle("Lidar Output");
        Group root = new Group();
        Canvas canvas = new Canvas(500, 500);
        GraphicsContext gc = canvas.getGraphicsContext2D();
        drawShapes(gc);
        root.getChildren().add(canvas);
        primaryStage.setScene(new Scene(root));
        primaryStage.show();
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
                System.out.println(Arrays.toString((float[]) object));
            }
        });
        new Thread(client).start();

        client.connect(4000, host, tcpPort);
    }

    private void drawShapes(GraphicsContext gc) {
        gc.setFill(Color.RED);
        gc.setStroke(Color.BLACK);
        gc.fillRoundRect(10, 110, 30, 30, 20, 25);
    }

    public float[] getCartesian(float[] polar) {
        return new float[]{(float) (polar[1] * Math.cos(polar[0])), (float) (polar[1] * Math.sin(polar[0]))};
    }
}
