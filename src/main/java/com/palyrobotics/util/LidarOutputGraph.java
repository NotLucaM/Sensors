package com.palyrobotics.util;

import com.esotericsoftware.kryonet.Client;
import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.stage.Stage;

import java.io.IOException;
import java.util.Arrays;

public class LidarOutputGraph extends Application {

    private final String host = "";
    private final int tcpPort = 5807;

    private Group root;

    private int circleNumber = 0;
    private int maxCircles = 750;

    public static void main(String[] args) {
        launch(args);
    }

    @Override
    public void start(Stage primaryStage) throws Exception {
        primaryStage.setTitle("Lidar Output");
        root = new Group();
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
                    Platform.runLater(() ->
                        drawShapes((float[]) object));
                };
            }
        });
        new Thread(client).start();

        client.connect(4000, host, tcpPort);
    }

    private void drawShapes(float[] polar) {
        float[] cartesian = getCartesian(polar);

        root.getChildren().add(new Circle(cartesian[0] / 5 + 500, cartesian[1] / 5 + 500, 10));
        circleNumber += 1;
        if (circleNumber > maxCircles) {
            root.getChildren().remove(0);
        }
    }

    public float[] getCartesian(float[] polar) { // float array is {angle, distance}, cartesian is {x, y} TODO: make more efficient by not creating a new array every time
        return new float[]{(float) (polar[1] * Math.cos(Math.toRadians(polar[0]))),
                (float) (polar[1] * Math.sin(Math.toRadians(polar[0])))};
    }
}
