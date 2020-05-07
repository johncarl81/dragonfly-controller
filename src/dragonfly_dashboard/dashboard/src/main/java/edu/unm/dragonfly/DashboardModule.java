package edu.unm.dragonfly;

import com.google.inject.AbstractModule;
import org.ros.node.ConnectedNode;

public class DashboardModule extends AbstractModule {

    private static ConnectedNode node;

    public static void setNode(ConnectedNode node) {
        DashboardModule.node = node;
    }

    @Override
    protected void configure() {
        bind(ConnectedNode.class).toInstance(node);
    }
}
