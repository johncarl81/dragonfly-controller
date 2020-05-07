package edu.unm.dragonfly;

import javafx.application.Application;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

public class Dashboard extends AbstractNodeMain {

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("dragonfly/dashboard");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {

        DashboardModule.setNode(connectedNode);

        Application.launch(DashboardApplication.class);
    }
}
