package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;
import com.esri.arcgisruntime.mapping.ArcGISScene;
import com.esri.arcgisruntime.mapping.ArcGISTiledElevationSource;
import com.esri.arcgisruntime.mapping.Basemap;
import com.esri.arcgisruntime.mapping.Surface;
import com.esri.arcgisruntime.mapping.view.Camera;
import com.esri.arcgisruntime.mapping.view.Graphic;
import com.esri.arcgisruntime.mapping.view.GraphicsOverlay;
import com.esri.arcgisruntime.mapping.view.SceneView;
import com.esri.arcgisruntime.symbology.CompositeSymbol;
import com.esri.arcgisruntime.symbology.SceneSymbol;
import com.esri.arcgisruntime.symbology.SimpleMarkerSceneSymbol;
import com.esri.arcgisruntime.symbology.TextSymbol;
import com.sun.tools.javac.util.List;
import io.reactivex.Observer;
import io.reactivex.disposables.Disposable;
import io.reactivex.functions.Consumer;
import io.reactivex.rxjavafx.schedulers.JavaFxScheduler;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ListView;
import javafx.scene.control.TextInputDialog;
import org.ros.node.ConnectedNode;
import sensor_msgs.NavSatFix;

import javax.inject.Inject;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Optional;

public class DashboardController {

    private static final DateFormat DATE_FORMAT = new SimpleDateFormat("hh:mm:ss");
    private static final String ELEVATION_IMAGE_SERVICE =
            "https://elevation3d.arcgis.com/arcgis/rest/services/WorldElevation3D/Terrain3D/ImageServer";

    @FXML
    private SceneView sceneView;
    @FXML
    private Button add;
    @FXML
    private Button delete;
    @FXML
    private Button center;
    @FXML
    private ListView<Drone> drones;
    @FXML
    private ListView<String> log;

    private final ObservableList<Drone> droneList = FXCollections.observableArrayList();
    private final ObservableList<String> logList = FXCollections.observableArrayList();
    private final GraphicsOverlay droneOverlay = new GraphicsOverlay();

    @Inject
    private ConnectedNode node;

    public void initialize() {
        // create a scene and add a basemap to it
        ArcGISScene scene = new ArcGISScene();
        scene.setBasemap(Basemap.createImagery());

        sceneView.setArcGISScene(scene);

        sceneView.getGraphicsOverlays().add(droneOverlay);


        // add base surface for elevation data
        Surface surface = new Surface();
        surface.getElevationSources().add(new ArcGISTiledElevationSource(ELEVATION_IMAGE_SERVICE));
        scene.setBaseSurface(surface);

        drones.setItems(droneList);
        log.setItems(logList);

        delete.setDisable(true);
        center.setDisable(true);

        add.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                TextInputDialog dialog = new TextInputDialog("Add Drone");
                dialog.setHeaderText("Add Drone");
                Optional<String> output = dialog.showAndWait();

                if(output.isPresent()) {
                    addDrone(output.get());
                    drones.getSelectionModel().clearSelection();
                }
            }
        });

        delete.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                deleteDrone(drones.getSelectionModel().getSelectedItem());
                drones.getSelectionModel().clearSelection();
            }
        });

        center.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                centerDrone(drones.getSelectionModel().getSelectedItem());
                drones.getSelectionModel().clearSelection();
            }
        });

        drones.getSelectionModel().selectedItemProperty().addListener(new ChangeListener<Drone>() {
            @Override
            public void changed(ObservableValue observable, Drone oldValue, Drone newValue) {
                boolean selected = newValue != null;
                delete.setDisable(!selected);
                center.setDisable(!selected);
            }
        });

        log("Dashboard Startup");
    }

    private void centerDrone(Drone drone) {
         drone.getLatestPosition()
                 .observeOn(JavaFxScheduler.platform())
                 .subscribe(new Consumer<NavSatFix>() {
                     @Override
                     public void accept(NavSatFix position) {
                         Camera camera = new Camera(position.getLatitude(), position.getLongitude(), 0, 0, 0, 0);
                         sceneView.setViewpointCameraAsync(camera);
                     }
                 });
    }

    private void deleteDrone(Drone name) {
        droneList.remove(name);
        name.shutdown();
        log("Removed " + name);
    }

    private void addDrone(String name) {

        Drone drone = new Drone(name);

        drone.init(node);

        drone.getPositions()
                .observeOn(JavaFxScheduler.platform())
                .subscribe(new Observer<NavSatFix>() {
                    Graphic droneGraphic;
                    @Override
                    public void onSubscribe(Disposable d) {}

                    @Override
                    public void onNext(NavSatFix navSatFix) {
                        Point point = new Point(navSatFix.getLongitude(), navSatFix.getLatitude(), navSatFix.getAltitude());
                        if (droneGraphic == null) {
                            SimpleMarkerSceneSymbol symbol = new SimpleMarkerSceneSymbol(SimpleMarkerSceneSymbol.Style.CYLINDER, 0xFFFF0000, 1, 1, 1, SceneSymbol.AnchorPosition.CENTER);
                            TextSymbol nameText = new TextSymbol(10, name, 0xFFFFFFFF, TextSymbol.HorizontalAlignment.LEFT, TextSymbol.VerticalAlignment.MIDDLE);
                            nameText.setOffsetX(25);
                            droneGraphic = new Graphic(point, new CompositeSymbol(List.of(symbol, nameText)));
                            droneOverlay.getGraphics().add(droneGraphic);
                        } else {
                            droneGraphic.setGeometry(point);
                        }
                    }

                    @Override
                    public void onError(Throwable e) {

                    }

                    @Override
                    public void onComplete() {
                        if(droneGraphic != null) {
                            droneOverlay.getGraphics().remove(droneGraphic);
                        }
                    }
                });

        droneList.add(drone);



        log("Added " + name);
    }

    private void log(String message) {
        logList.add("[" + DATE_FORMAT.format(new Date()) + "]" + message);
    }

    void terminate() {
        if (sceneView != null) {
            sceneView.dispose();
        }
    }
}
