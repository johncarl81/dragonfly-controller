package edu.unm.dragonfly;


import javafx.scene.control.*;
import javafx.scene.layout.GridPane;

import java.util.Optional;

public class LawnmowerDialogFactory {

    public enum Walk {
        WALK(1),
        RANGE(2);

        public final int id;

        Walk(int id) {
            this.id = id;
        }
    }

    public interface DialogCallback {
        void call(float stepLength, float altitude, int stacks, boolean walkBoundary, Walk selectedItem, float waittime, float distanceThreshold);
    }

    public static void create(DialogCallback callback) {

        Dialog<ButtonType> dialog = new Dialog<>();
        dialog.setTitle("Lawnmower Parameters");

        GridPane grid = new GridPane();

        TextField stepLengthField = new TextField();
        TextField stacksField = new TextField();
        TextField altitudeField = new TextField();
        CheckBox walkBoundaryField = new CheckBox();
        ComboBox<Walk> walkComboBox = new ComboBox<>();
        walkComboBox.getItems().addAll(Walk.values());
        TextField waitTimeField = new TextField();
        TextField distanceThreshold = new TextField();

        // Set Defaults
        stepLengthField.setText("1");
        stacksField.setText("1");
        altitudeField.setText("10");
        walkBoundaryField.setSelected(true);
        walkComboBox.getSelectionModel().select(Walk.WALK);
        waitTimeField.setText("3");
        distanceThreshold.setText("1");

        grid.add(new Label("Step Length: "), 1, 1);
        grid.add(stepLengthField, 2, 1);
        grid.add(new Label("Altitude: "), 1, 2);
        grid.add(altitudeField, 2, 2);
        grid.add(new Label("Stacks: "), 1, 3);
        grid.add(stacksField, 2, 3);
        grid.add(new Label("Walk Boundary: "), 1, 4);
        grid.add(walkBoundaryField, 2, 4);
        grid.add(new Label("Walk: "), 1, 5);
        grid.add(walkComboBox, 2, 5);
        grid.add(new Label("Wait Time: "), 1, 6);
        grid.add(waitTimeField, 2, 6);
        grid.add(new Label("Distance Threshold: "), 1, 7);
        grid.add(distanceThreshold, 2, 7);

        dialog.getDialogPane().setContent(grid);

        dialog.getDialogPane().getButtonTypes().add(ButtonType.CANCEL);
        dialog.getDialogPane().getButtonTypes().add(ButtonType.OK);

        Optional<ButtonType> result = dialog.showAndWait();

        if(result.isPresent() && result.get() == ButtonType.OK) {
            callback.call(Float.parseFloat(stepLengthField.getText()),
                    Float.parseFloat(altitudeField.getText()),
                    Integer.parseInt(stacksField.getText()),
                    walkBoundaryField.isSelected(),
                    walkComboBox.getSelectionModel().getSelectedItem(),
                    Float.parseFloat(waitTimeField.getText()),
                    Float.parseFloat(distanceThreshold.getText()));
        }
    }
}
