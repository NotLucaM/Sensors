package com.palyrobotics.util;

import javafx.application.Application;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

public class LidarOutputGraph extends Application {

    private final String address;
    private final int tcpPort;

    public LidarOutputGraph(String address, int tcpPort) {
        this.address = address;
        this.tcpPort = tcpPort;
    }


    @Override
    public void start(Stage primaryStage) throws Exception {
        primaryStage.setTitle("Lidar Output");
        Group root = new Group();
        Canvas canvas = new Canvas(500, 500);
        GraphicsContext gc = canvas.getGraphicsContext2D();
        drawShapes(gc);
        root.getChildren().add(canvas);
        primaryStage.setScene(new Scene(root));
        primaryStage.show();
    }

    public void drawShapes(GraphicsContext gc) {
        gc.setFill(Color.RED);
        gc.setStroke(Color.BLACK);
        gc.fillRoundRect(10, 110, 30, 30, 20, 25);
    }
}
