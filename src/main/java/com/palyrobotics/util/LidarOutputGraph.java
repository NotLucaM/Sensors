package com.palyrobotics.util;

import com.esotericsoftware.kryonet.Client;
import com.esotericsoftware.kryonet.Connection;
import com.esotericsoftware.kryonet.Listener;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.shape.Circle;
import javafx.stage.Stage;

import java.io.IOException;

public class LidarOutputGraph extends Application {

    // i guess naming conventions are a thing
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
                if (object instanceof float[]) { // Filters out keepAlive messages
                    Platform.runLater(() ->
                        drawShape((float[]) object)); // Still will send float arrays even if there is a point class because I believe they are more versatile
                };
            }
        });
        new Thread(client).start();

        client.connect(4000, host, tcpPort);
    }

    private void drawShape(float[] polar) {
        float[] cartesian = getCartesian(polar); // TODO: make more efficient

        root.getChildren().add(new Circle(cartesian[0] / 5 + 750, cartesian[1] / 5 + 750, 10));
        circleNumber += 1;
        if (circleNumber > maxCircles) { // Deletes the oldest point after a certain amount of points are there
            root.getChildren().remove(0);
        }
    }

    // TODO: Maybe I should use points here, but I use float arrays everywhere else in this class so idk?
    public float[] getCartesian(float[] polar) { // float array is {angle, distance}, cartesian is {x, y} TODO: make more efficient by not creating a new array every time
        return new float[]{(float) (polar[1] * Math.cos(Math.toRadians(polar[0]))),
                (float) (polar[1] * Math.sin(Math.toRadians(polar[0])))};
    }
}
