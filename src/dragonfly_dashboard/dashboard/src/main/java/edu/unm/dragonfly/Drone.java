package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;
import dragonfly_messages.Lawnmower;
import dragonfly_messages.LawnmowerRequest;
import dragonfly_messages.LawnmowerResponse;
import io.reactivex.Observable;
import io.reactivex.subjects.BehaviorSubject;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;
import sensor_msgs.NavSatFix;
import std_srvs.Empty;
import std_srvs.EmptyResponse;

import java.util.List;
import java.util.stream.Collectors;

public class Drone {

    private final String name;
    private final ConnectedNode node;
    private Subscriber<NavSatFix> subscriber;
    private final BehaviorSubject<NavSatFix> position = BehaviorSubject.create();

    public Drone(ConnectedNode node, String name) {
        this.node = node;
        this.name = name;
    }

    public void init() throws ServiceNotFoundException {
        subscriber = node.newSubscriber(name + "/mavros/global_position/global", NavSatFix._TYPE);
        subscriber.addMessageListener(new MessageListener<NavSatFix>() {
            @Override
            public void onNewMessage(NavSatFix navSatFix) {
                position.onNext(navSatFix);
            }
        });
    }

    public void send(List<Point> points) throws ServiceNotFoundException {
        ServiceClient<LawnmowerRequest, LawnmowerResponse> client = node.newServiceClient(name + "/command/lawnmower", Lawnmower._TYPE);
        LawnmowerRequest request = client.newMessage();

        NodeConfiguration config = NodeConfiguration.newPrivate();

        request.setAltitude(10);
        request.setBoundary(points.stream().map(input -> {
                    NavSatFix navSatFix = config.getTopicMessageFactory().newFromType(NavSatFix._TYPE);
                    navSatFix.setLatitude(input.getY());
                    navSatFix.setLongitude(input.getX());
                    return navSatFix;
                }).collect(Collectors.toList())
        );
        request.setSteplength(1.0f);
        client.call(request, new ServiceResponseListener<LawnmowerResponse>() {
             @Override
             public void onSuccess(LawnmowerResponse lawnmowerResponse) {
                 System.out.println("Got: " + lawnmowerResponse.getMessage());
             }

             @Override
             public void onFailure(RemoteException e) {

             }
         });

    }

    public void sendDDSA() throws ServiceNotFoundException {
        ServiceClient<Empty, EmptyResponse> client = node.newServiceClient(name + "/command/ddsa", Empty._TYPE);
        Empty request = client.newMessage();
        client.call(request, new ServiceResponseListener<EmptyResponse>() {
            @Override
            public void onSuccess(EmptyResponse response) {
                System.out.println("Got: " + response.toString());
            }

            @Override
            public void onFailure(RemoteException e) {

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
