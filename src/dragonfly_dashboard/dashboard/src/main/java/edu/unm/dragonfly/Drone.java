package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;
import dragonfly_messages.Lawnmower;
import dragonfly_messages.LawnmowerRequest;
import dragonfly_messages.LawnmowerResponse;
import geometry_msgs.PoseStamped;
import io.reactivex.Observable;
import io.reactivex.functions.BiFunction;
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
import std_srvs.EmptyRequest;
import std_srvs.EmptyResponse;

import java.util.List;
import java.util.stream.Collectors;

public class Drone {

    private final String name;
    private final ConnectedNode node;
    private Subscriber<NavSatFix> subscriber;
    private Subscriber<PoseStamped> localPositionSubscriber;
    private final BehaviorSubject<NavSatFix> position = BehaviorSubject.create();
    private final BehaviorSubject<PoseStamped> localPosition = BehaviorSubject.create();
    private final Observable<LatLonRelativeAltitude> relativeAltitudeObservable;

    public Drone(ConnectedNode node, String name) {
        this.node = node;
        this.name = name;

        relativeAltitudeObservable = Observable.combineLatest(position, localPosition, new BiFunction<NavSatFix, PoseStamped, LatLonRelativeAltitude>() {
            @Override
            public LatLonRelativeAltitude apply(NavSatFix navSatFix, PoseStamped poseStamped) throws Exception {
                return new LatLonRelativeAltitude(navSatFix.getLatitude(), navSatFix.getLongitude(), poseStamped.getPose().getPosition().getZ());
            }
        });
    }

    public void init() throws ServiceNotFoundException {

        subscriber = node.newSubscriber(name + "/mavros/global_position/global", NavSatFix._TYPE);
        subscriber.addMessageListener(new MessageListener<NavSatFix>() {
            @Override
            public void onNewMessage(NavSatFix navSatFix) {
                position.onNext(navSatFix);
            }
        });

        localPositionSubscriber = node.newSubscriber(name + "/mavros/local_position/pose", PoseStamped._TYPE);
        localPositionSubscriber.addMessageListener(new MessageListener<PoseStamped>() {
            @Override
            public void onNewMessage(PoseStamped pose) {
                localPosition.onNext(pose);
            }
        });
    }

    public void lawnmower(List<Point> points) throws ServiceNotFoundException {
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

    public void ddsa() throws ServiceNotFoundException {
        ServiceClient<Object, EmptyResponse> client = node.newServiceClient(name + "/command/ddsa", EmptyRequest._TYPE);
        Object request = client.newMessage();
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

    public Observable<LatLonRelativeAltitude> getLatestPosition() {
        return relativeAltitudeObservable.take(1);
    }

    public Observable<LatLonRelativeAltitude> getPositions() {
        return relativeAltitudeObservable;
    }

    public void shutdown() {
        subscriber.shutdown();
        localPositionSubscriber.shutdown();
        position.onComplete();
        localPosition.onComplete();
    }

    public static class LatLonRelativeAltitude {
        private final double latitude;
        private final double longitude;
        private final double relativeAltitude;

        public LatLonRelativeAltitude(double latitude, double longitude, double relativeAltitude) {
            this.latitude = latitude;
            this.longitude = longitude;
            this.relativeAltitude = relativeAltitude;
        }

        public double getLatitude() {
            return latitude;
        }

        public double getLongitude() {
            return longitude;
        }

        public double getRelativeAltitude() {
            return relativeAltitude;
        }
    }
}
