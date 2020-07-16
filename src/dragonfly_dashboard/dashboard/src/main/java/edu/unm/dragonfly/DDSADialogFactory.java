package edu.unm.dragonfly;

import javafx.scene.control.*;
import javafx.scene.layout.GridPane;

import java.util.Optional;

public class DDSADialogFactory {

    public interface DialogCallback {
        void call(float radius, float stepLength, float altitude, int loops, int stacks, Walk walk, float waittime, float distanceThreshold);
    }

    public static void create(DialogCallback callback) {

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("DDSA Parameters");

        GridPane grid = new GridPane();

        TextField radiusField = new TextField();
        TextField stepLengthField = new TextField();
        TextField loopsField = new TextField();
        TextField stacksField = new TextField();
        TextField altitudeField = new TextField();
        ComboBox<Walk> walkComboBox = new ComboBox<>();
        walkComboBox.getItems().addAll(Walk.values());
        TextField waitTimeField = new TextField();
        TextField distanceThreshold = new TextField();

        // Set Defaults
        radiusField.setText("1");
        stepLengthField.setText("1");
        loopsField.setText("5");
        stacksField.setText("1");
        altitudeField.setText("10");
        walkComboBox.getSelectionModel().select(Walk.WALK);
        waitTimeField.setText("3");
        distanceThreshold.setText("1");

        grid.add(new Label("Radius: "), 1, 1);
        grid.add(radiusField, 2, 1);
        grid.add(new Label("Step Length: "), 1, 2);
        grid.add(stepLengthField, 2, 2);
        grid.add(new Label("Altitude: "), 1, 3);
        grid.add(altitudeField, 2, 3);
        grid.add(new Label("Loops: "), 1, 4);
        grid.add(loopsField, 2, 4);
        grid.add(new Label("Stacks: "), 1, 5);
        grid.add(stacksField, 2, 5);
        grid.add(new Label("Walk: "), 1, 6);
        grid.add(walkComboBox, 2, 6);
        grid.add(new Label("Wait Time: "), 1, 7);
        grid.add(waitTimeField, 2, 7);
        grid.add(new Label("Distance Threshold: "), 1, 8);
        grid.add(distanceThreshold, 2, 8);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(Float.parseFloat(radiusField.getText()),
                    Float.parseFloat(stepLengthField.getText()),
                    Float.parseFloat(altitudeField.getText()),
                    Integer.parseInt(loopsField.getText()),
                    Integer.parseInt(stacksField.getText()),
                    walkComboBox.getSelectionModel().getSelectedItem(),
                    Float.parseFloat(waitTimeField.getText()),
                    Float.parseFloat(distanceThreshold.getText()));
        }
    }
}
