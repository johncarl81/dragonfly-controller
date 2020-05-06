package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;
import com.esri.arcgisruntime.geometry.SpatialReference;
import com.esri.arcgisruntime.mapping.ArcGISScene;
import com.esri.arcgisruntime.mapping.ArcGISTiledElevationSource;
import com.esri.arcgisruntime.mapping.Basemap;
import com.esri.arcgisruntime.mapping.Surface;
import com.esri.arcgisruntime.mapping.view.Camera;
import com.esri.arcgisruntime.mapping.view.SceneView;
import javafx.fxml.FXML;

public class DashboardController {

    private static final String ELEVATION_IMAGE_SERVICE =
            "https://elevation3d.arcgis.com/arcgis/rest/services/WorldElevation3D/Terrain3D/ImageServer";

    @FXML
    private SceneView sceneView;

    public void initialize() {
        // create a scene and add a basemap to it
        ArcGISScene scene = new ArcGISScene();
        scene.setBasemap(Basemap.createImagery());

        sceneView.setArcGISScene(scene);


        // add base surface for elevation data
        Surface surface = new Surface();
        surface.getElevationSources().add(new ArcGISTiledElevationSource(ELEVATION_IMAGE_SERVICE));
        scene.setBaseSurface(surface);

        // add a camera and initial camera position
        //      Camera camera = new Camera(28.4, 83.9, 10010.0, 10.0, 80.0, 0.0);
        //      Camera camera = new Camera(-22.029, -67.244, 7120, 56, 68, 0.0);
        Camera camera = new Camera(new Point(3931279.465, 333511.130, 1000, SpatialReference.create(4326)), 0, 0, 0);

        sceneView.setViewpointCamera(camera);
    }

    void terminate() {
        if (sceneView != null) {
            sceneView.dispose();
        }
    }
}
