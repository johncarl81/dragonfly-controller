package edu.unm.dragonfly;

import javafx.scene.control.*;
import javafx.scene.layout.GridPane;

import java.util.Optional;

public class RandomPathDialogFactory {

    public interface DialogCallback {
        void call(float minAltitude, float maxAltitude, int size, int iterations, int population, float waittime, float distanceThreshold);
    }

    public static void create(DialogCallback callback) {

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Random Path Parameters");

        GridPane grid = new GridPane();

        TextField minAltitude = new TextField();
        TextField maxAltitude = new TextField();
        TextField size = new TextField();
        TextField iterations = new TextField();
        TextField population = new TextField();
        TextField waitTimeField = new TextField();
        TextField distanceThreshold = new TextField();
        ProgressBar progressBar = new ProgressBar(0);

        // Set Defaults
        minAltitude.setText("10");
        maxAltitude.setText("20");
        size.setText("100");
        iterations.setText("100");
        population.setText("100");
        waitTimeField.setText("3");
        distanceThreshold.setText("1");

        grid.add(new Label("Min Altitude: "), 1, 1);
        grid.add(minAltitude, 2, 1);
        grid.add(new Label("Max Altitude: "), 1, 2);
        grid.add(maxAltitude, 2, 2);
        grid.add(new Label("Size: "), 1, 3);
        grid.add(size, 2, 3);
        grid.add(new Label("Iterations: "), 1, 4);
        grid.add(iterations, 2, 4);
        grid.add(new Label("Population: "), 1, 5);
        grid.add(population, 2, 5);
        grid.add(new Label("Wait Time: "), 1, 6);
        grid.add(waitTimeField, 2, 6);
        grid.add(new Label("Distance Threshold: "), 1, 7);
        grid.add(distanceThreshold, 2, 7);
        grid.add(progressBar, 1, 8, 2, 8);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(Float.parseFloat(minAltitude.getText()),
                    Float.parseFloat(maxAltitude.getText()),
                    Integer.parseInt(size.getText()),
                    Integer.parseInt(iterations.getText()),
                    Integer.parseInt(population.getText()),
                    Float.parseFloat(waitTimeField.getText()),
                    Float.parseFloat(distanceThreshold.getText()));
        }
    }
}
