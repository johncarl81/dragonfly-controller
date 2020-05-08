package edu.unm.dragonfly;

import io.reactivex.Observable;
import io.reactivex.subjects.BehaviorSubject;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;
import sensor_msgs.NavSatFix;

public class Drone {

    private final String name;
    Subscriber<NavSatFix> subscriber;
    private BehaviorSubject<NavSatFix> position = BehaviorSubject.create();

    public Drone(String name) {
        this.name = name;
    }

    public void init(ConnectedNode node) {
        subscriber = node.newSubscriber(name + "/mavros/global_position/global", sensor_msgs.NavSatFix._TYPE);
        subscriber.addMessageListener(new MessageListener<NavSatFix>() {
            @Override
            public void onNewMessage(NavSatFix navSatFix) {
                position.onNext(navSatFix);
            }
        });
    }

    @Override
    public String toString() {
        return name;
    }

    public Observable<NavSatFix> getLatestPosition() {
        return position.take(1);
    }

    public Observable<NavSatFix> getPositions() {
        return position;
    }

    public void shutdown() {
        subscriber.shutdown();
        position.onComplete();
    }
}
