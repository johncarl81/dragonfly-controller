package edu.unm.dragonfly;

import com.esri.arcgisruntime.concurrent.ListenableFuture;
import com.esri.arcgisruntime.geometry.*;
import com.esri.arcgisruntime.mapping.ArcGISScene;
import com.esri.arcgisruntime.mapping.ArcGISTiledElevationSource;
import com.esri.arcgisruntime.mapping.Basemap;
import com.esri.arcgisruntime.mapping.Surface;
import com.esri.arcgisruntime.mapping.view.*;
import com.esri.arcgisruntime.symbology.*;
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
import javafx.geometry.Point2D;
import javafx.scene.control.Button;
import javafx.scene.control.ListView;
import javafx.scene.control.TextField;
import javafx.scene.control.TextInputDialog;
import javafx.scene.input.MouseEvent;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;

import javax.inject.Inject;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.ExecutionException;

public class DashboardController {

    private static final DateFormat DATE_FORMAT = new SimpleDateFormat("hh:mm:ss");
    private static final String ELEVATION_IMAGE_SERVICE =
            "https://elevation3d.arcgis.com/arcgis/rest/services/WorldElevation3D/Terrain3D/ImageServer";
    private static final int ALPHA_RED = 0x33FF0000;
    private static final int RED = 0xFFFF0000;

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
    @FXML
    private TextField coordinates;
    @FXML
    private Button select;
    @FXML
    private Button lawnmower;
    @FXML
    private Button ddsa;
    @FXML
    private Button cancel;

    private final ObservableList<Drone> droneList = FXCollections.observableArrayList();
    private final ObservableList<String> logList = FXCollections.observableArrayList();
    private final GraphicsOverlay droneOverlay = new GraphicsOverlay();
    private final GraphicsOverlay droneShadowOverlay = new GraphicsOverlay();
    private final GraphicsOverlay boundaryOverlay = new GraphicsOverlay();
    private final List<Point> boundaryPoints = new ArrayList<Point>();
    private CoordianteSelectionMode mode = CoordianteSelectionMode.CLEAR;

    private enum CoordianteSelectionMode {
        SELECT("Finished"),
        FINISHED("Clear"),
        CLEAR("Select Boundary");

        private final String buttonLabel;

        CoordianteSelectionMode(String buttonLabel) {
            this.buttonLabel = buttonLabel;
        }
    }

    @Inject
    private ConnectedNode node;

    public void initialize() {
        // create a scene and add a basemap to it
        ArcGISScene scene = new ArcGISScene();
        scene.setBasemap(Basemap.createImagery());

        sceneView.setArcGISScene(scene);

        sceneView.getGraphicsOverlays().add(droneOverlay);
        droneOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.RELATIVE_TO_SCENE);
        sceneView.getGraphicsOverlays().add(droneShadowOverlay);
        droneShadowOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.DRAPED_FLAT);
        sceneView.getGraphicsOverlays().add(boundaryOverlay);
        boundaryOverlay.getSceneProperties().setSurfacePlacement(LayerSceneProperties.SurfacePlacement.DRAPED_FLAT);

        sceneView.setOnMouseMoved(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent event) {
                Point2D point2D = new Point2D(event.getX(), event.getY());

                // get the scene location from the screen position
                ListenableFuture<Point> pointFuture = sceneView.screenToLocationAsync(point2D);
                pointFuture.addDoneListener(() -> {
                    try {
                        Point point = pointFuture.get();
                        coordinates.setText("Lat: " + point.getY() + " Lon: " + point.getX());

                    } catch (InterruptedException | ExecutionException e) {
                        e.printStackTrace();
                    }
                });
            }
        });

        sceneView.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent event) {
                if(mode == CoordianteSelectionMode.SELECT) {
                    Point2D point2D = new Point2D(event.getX(), event.getY());

                    // get the scene location from the screen position
                    ListenableFuture<Point> pointFuture = sceneView.screenToLocationAsync(point2D);
                    pointFuture.addDoneListener(() -> {
                        try {
                            Point point = pointFuture.get();

                            boundaryPoints.add(point);

                            boundaryOverlay.getGraphics().clear();

                            Graphic boudaryGraphic;
                            if(boundaryPoints.size() == 1) {
                                SimpleMarkerSymbol markerSymbol = new SimpleMarkerSymbol(SimpleMarkerSymbol.Style.CIRCLE, RED, 2);
                                boudaryGraphic = new Graphic(new Point(boundaryPoints.get(0).getX(), boundaryPoints.get(0).getY()), markerSymbol);
                            } else if(boundaryPoints.size() == 2) {
                                PolylineBuilder lineBuilder = new PolylineBuilder(SpatialReferences.getWgs84());
                                lineBuilder.addPoint(boundaryPoints.get(0).getX(), boundaryPoints.get(0).getY());
                                lineBuilder.addPoint(boundaryPoints.get(1).getX(), boundaryPoints.get(1).getY());
                                SimpleLineSymbol lineSymbol = new SimpleLineSymbol(SimpleLineSymbol.Style.SOLID, RED, 2);
                                boudaryGraphic = new Graphic(lineBuilder.toGeometry(), lineSymbol);
                            } else {
                                PointCollection polygonPoints = new PointCollection(boundaryPoints);
                                SimpleFillSymbol polygonSymbol = new SimpleFillSymbol(SimpleFillSymbol.Style.SOLID, ALPHA_RED, new SimpleLineSymbol(SimpleLineSymbol.Style.SOLID, RED, .3f));
                                boudaryGraphic = new Graphic(new Polygon(polygonPoints), polygonSymbol);
                            }
                            boundaryOverlay.getGraphics().add(boudaryGraphic);


                        } catch (InterruptedException | ExecutionException e) {
                            e.printStackTrace();
                        }
                    });
                }
            }
        });

        select.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                if(mode == CoordianteSelectionMode.SELECT) {
                    mode = CoordianteSelectionMode.FINISHED;
                    lawnmower.setDisable(!(!drones.getSelectionModel().isEmpty() && !boundaryPoints.isEmpty()));
                } else if (mode == CoordianteSelectionMode.FINISHED) {
                    mode = CoordianteSelectionMode.CLEAR;
                    boundaryOverlay.getGraphics().clear();
                    boundaryPoints.clear();
                } else if (mode == CoordianteSelectionMode.CLEAR) {
                    mode = CoordianteSelectionMode.SELECT;
                }
                select.setText(mode.buttonLabel);
            }
        });

        // add base surface for elevation data
        Surface surface = new Surface();
        surface.getElevationSources().add(new ArcGISTiledElevationSource(ELEVATION_IMAGE_SERVICE));
        scene.setBaseSurface(surface);

        drones.setItems(droneList);
        log.setItems(logList);

        delete.setDisable(true);
        center.setDisable(true);
        lawnmower.setDisable(true);
        ddsa.setDisable(true);
        cancel.setDisable(true);

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

        lawnmower.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                if(!boundaryPoints.isEmpty()) {
                        LawnmowerDialogFactory.create((stepLength, altitude, stacks, walkBoundary, walk, waittime) -> {
                            try {
                                drones.getSelectionModel().getSelectedItem().lawnmower(boundaryPoints, stepLength, altitude, stacks, walkBoundary, walk.id, waittime);
                            } catch (ServiceNotFoundException e) {
                                e.printStackTrace();
                            }
                        });
                }
                drones.getSelectionModel().clearSelection();
            }
        });

        ddsa.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                DDSADialogFactory.create((radius, stepLength, altitude, loops, stacks, walk, waittime) -> {
                    try {
                        drones.getSelectionModel().getSelectedItem().ddsa(radius, stepLength, altitude, loops, stacks, walk.id, waittime);
                    } catch (ServiceNotFoundException e) {
                        e.printStackTrace();
                    }
                });
                drones.getSelectionModel().clearSelection();
            }
        });

        cancel.setOnAction(new EventHandler<ActionEvent>() {
            @Override
            public void handle(ActionEvent event) {
                try {
                    drones.getSelectionModel().getSelectedItem().cancel();
                } catch (ServiceNotFoundException e) {
                    e.printStackTrace();
                }
                drones.getSelectionModel().clearSelection();
            }
        });

        drones.getSelectionModel().selectedItemProperty().addListener(new ChangeListener<Drone>() {
            @Override
            public void changed(ObservableValue observable, Drone oldValue, Drone newValue) {
                boolean selected = newValue != null;
                delete.setDisable(!selected);
                center.setDisable(!selected);
                lawnmower.setDisable(!(selected && !boundaryPoints.isEmpty() && mode == CoordianteSelectionMode.FINISHED));
                ddsa.setDisable(!selected);
                cancel.setDisable(!selected);
            }
        });

        log("Dashboard Startup");
    }

    private void centerDrone(Drone drone) {
         drone.getLatestPosition()
                 .observeOn(JavaFxScheduler.platform())
                 .subscribe(new Consumer<Drone.LatLonRelativeAltitude>() {
                     @Override
                     public void accept(Drone.LatLonRelativeAltitude position) {
                         Camera camera = new Camera(position.getLatitude(), position.getLongitude(), 10, 0, 0, 0);
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

        Drone drone = new Drone(node, name);
        drone.init();

        drone.getLog()
                .observeOn(JavaFxScheduler.platform())
                .subscribe(message -> log(name + ": " + message));

        drone.getPositions()
                .observeOn(JavaFxScheduler.platform())
                .subscribe(new Observer<Drone.LatLonRelativeAltitude>() {
                    private Graphic droneGraphic;
                    private Graphic droneShadowGraphic;
                    @Override
                    public void onSubscribe(Disposable d) {}

                    @Override
                    public void onNext(Drone.LatLonRelativeAltitude navSatFix) {
                        Point point = new Point(navSatFix.getLongitude(), navSatFix.getLatitude(), navSatFix.getRelativeAltitude());
                        if (droneGraphic == null) {
                            SimpleMarkerSceneSymbol symbol = new SimpleMarkerSceneSymbol(SimpleMarkerSceneSymbol.Style.CYLINDER, 0xFFFF0000, 1, 1, 1, SceneSymbol.AnchorPosition.CENTER);
                            TextSymbol nameText = new TextSymbol(10, name, 0xFFFFFFFF, TextSymbol.HorizontalAlignment.LEFT, TextSymbol.VerticalAlignment.MIDDLE);
                            nameText.setOffsetX(25);
                            droneGraphic = new Graphic(point, new CompositeSymbol(Arrays.asList(symbol, nameText)));
                            droneOverlay.getGraphics().add(droneGraphic);

                            SimpleMarkerSymbol shadowSymbol = new SimpleMarkerSymbol(SimpleMarkerSymbol.Style.CIRCLE, 0x99000000, 2.5f);
                            droneShadowGraphic = new Graphic(point, shadowSymbol);
                            droneShadowOverlay.getGraphics().add(droneShadowGraphic);
                        } else {
                            droneGraphic.setGeometry(point);
                            droneShadowGraphic.setGeometry(point);
                        }
                    }

                    @Override
                    public void onError(Throwable e) {

                    }

                    @Override
                    public void onComplete() {
                        if(droneGraphic != null) {
                            droneOverlay.getGraphics().remove(droneGraphic);
                            droneShadowOverlay.getGraphics().remove(droneShadowGraphic);
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
