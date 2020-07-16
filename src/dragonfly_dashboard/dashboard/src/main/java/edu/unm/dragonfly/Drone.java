package edu.unm.dragonfly;

import com.esri.arcgisruntime.geometry.Point;
import dragonfly_messages.*;
import geometry_msgs.PoseStamped;
import io.reactivex.Observable;
import io.reactivex.Single;
import io.reactivex.subjects.BehaviorSubject;
import io.reactivex.subjects.PublishSubject;
import io.reactivex.subjects.SingleSubject;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Subscriber;
import sensor_msgs.NavSatFix;
import std_srvs.EmptyRequest;
import std_srvs.EmptyResponse;

import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

public class Drone {

    private final String name;
    private final ConnectedNode node;
    private Subscriber<NavSatFix> subscriber;
    private Subscriber<PoseStamped> localPositionSubscriber;
    private Subscriber<std_msgs.String> logSubscriber;
    private final BehaviorSubject<NavSatFix> position = BehaviorSubject.create();
    private final BehaviorSubject<PoseStamped> localPosition = BehaviorSubject.create();
    private final PublishSubject<String> logSubject = PublishSubject.create();
    private final Observable<LatLonRelativeAltitude> relativeAltitudeObservable;

    public Drone(ConnectedNode node, String name) {
        this.node = node;
        this.name = name;

        relativeAltitudeObservable = Observable.combineLatest(position, localPosition,
                (navSatFix, poseStamped) -> new LatLonRelativeAltitude(navSatFix.getLatitude(), navSatFix.getLongitude(), poseStamped.getPose().getPosition().getZ()));
    }

    public void init() {

        subscriber = node.newSubscriber(name + "/mavros/global_position/global", NavSatFix._TYPE);
        subscriber.addMessageListener(position::onNext);

        localPositionSubscriber = node.newSubscriber(name + "/mavros/local_position/pose", PoseStamped._TYPE);
        localPositionSubscriber.addMessageListener(localPosition::onNext);

        logSubscriber = node.newSubscriber(name + "/log", std_msgs.String._TYPE);
        logSubscriber.addMessageListener(message -> logSubject.onNext(message.getData()));
    }

    public void lawnmower(List<Point> boundaryPoints, float stepLength, float altitude, int stacks, boolean walkBoundary, int walk, float waittime, float distanceThreshold) throws ServiceNotFoundException {
        ServiceClient<LawnmowerRequest, LawnmowerResponse> client = node.newServiceClient(name + "/command/lawnmower", Lawnmower._TYPE);
        LawnmowerRequest request = client.newMessage();

        NodeConfiguration config = NodeConfiguration.newPrivate();

        request.setBoundary(boundaryPoints.stream().map(mapToLatLon(config)).collect(Collectors.toList()));
        request.setSteplength(stepLength);
        request.setWalkBoundary(walkBoundary);
        request.setStacks(stacks);
        request.setAltitude(altitude);
        request.setWalk(walk);
        request.setWaittime(waittime);
        request.setDistanceThreshold(distanceThreshold);

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

    public Single<List<Point>> getLawnmowerWaypoints(List<Point> boundaryPoints, float stepLength, float altitude, int stacks, boolean walkBoundary, int walk, float waittime) throws ServiceNotFoundException {
        ServiceClient<LawnmowerWaypointsRequest, LawnmowerWaypointsResponse> client = node.newServiceClient(name + "/build/lawnmower", LawnmowerWaypoints._TYPE);
        LawnmowerWaypointsRequest request = client.newMessage();

        NodeConfiguration config = NodeConfiguration.newPrivate();

        request.setBoundary(boundaryPoints.stream().map(mapToLatLon(config)).collect(Collectors.toList())
        );
        request.setSteplength(stepLength);
        request.setWalkBoundary(walkBoundary);
        request.setStacks(stacks);
        request.setAltitude(altitude);
        request.setWalk(walk);
        request.setWaittime(waittime);

        SingleSubject<List<Point>> result = SingleSubject.create();

        client.call(request, new ServiceResponseListener<LawnmowerWaypointsResponse>() {
            @Override
            public void onSuccess(LawnmowerWaypointsResponse response) {
                result.onSuccess(response.getWaypoints().stream().map(mapToPoint()).collect(Collectors.toList()));
            }

            @Override
            public void onFailure(RemoteException e) {

            }
        });

        return result;
    }

    private Function<Point, LatLon> mapToLatLon(NodeConfiguration config) {
        return input -> {
            LatLon position = config.getTopicMessageFactory().newFromType(LatLon._TYPE);
            position.setLatitude(input.getY());
            position.setLongitude(input.getX());
            position.setRelativeAltitude(input.getZ());
            return position;
        };
    }

    private Function<LatLon, Point> mapToPoint() {
        return input -> {
            System.out.println(new Point(input.getLongitude(), input.getLatitude(), input.getRelativeAltitude()));
            return new Point(input.getLongitude(), input.getLatitude(), input.getRelativeAltitude());
        };
    }

    public void ddsa(float radius, float stepLength, float altitude, int loops, int stacks, int walk, float waittime, float distanceThreshold) throws ServiceNotFoundException {
        ServiceClient<DDSARequest, DDSAResponse> client = node.newServiceClient(name + "/command/ddsa", DDSA._TYPE);
        DDSARequest request = client.newMessage();
        request.setRadius(radius);
        request.setSteplength(stepLength);
        request.setStacks(stacks);
        request.setAltitude(altitude);
        request.setWalk(walk);
        request.setWaittime(waittime);
        request.setLoops(loops);
        request.setDistanceThreshold(distanceThreshold);

        client.call(request, new ServiceResponseListener<DDSAResponse>() {
            @Override
            public void onSuccess(DDSAResponse response) {
                System.out.println("Got: " + response.toString());
            }

            @Override
            public void onFailure(RemoteException e) {

            }
        });

    }

    public Single<List<Point>> getDDSAWaypoints(float radius, float stepLength, float altitude, int loops, int stacks, int walk, float waittime) throws ServiceNotFoundException {
        ServiceClient<DDSAWaypointsRequest, DDSAWaypointsResponse> client = node.newServiceClient(name + "/build/ddsa", DDSAWaypoints._TYPE);
        DDSAWaypointsRequest request = client.newMessage();
        request.setRadius(radius);
        request.setSteplength(stepLength);
        request.setStacks(stacks);
        request.setAltitude(altitude);
        request.setWalk(walk);
        request.setWaittime(waittime);
        request.setLoops(loops);
        
        SingleSubject<List<Point>> result = SingleSubject.create();

        client.call(request, new ServiceResponseListener<DDSAWaypointsResponse>() {
            @Override
            public void onSuccess(DDSAWaypointsResponse response) {
                result.onSuccess(response.getWaypoints().stream().map(mapToPoint()).collect(Collectors.toList()));
            }

            @Override
            public void onFailure(RemoteException e) {

            }
        });
        
        return result;
    }

    public void navigate(List<Point> waypoints, float distanceThreshold) throws ServiceNotFoundException {
        ServiceClient<NavigationRequest, NavigationResponse> client = node.newServiceClient(name + "/command/navigate", Navigation._TYPE);
        NavigationRequest request = client.newMessage();

        NodeConfiguration config = NodeConfiguration.newPrivate();

        request.setWaypoints(waypoints.stream().map(mapToLatLon(config)).collect(Collectors.toList()));
        request.setWaittime(0);
        request.setDistanceThreshold(distanceThreshold);

        client.call(request, new ServiceResponseListener<NavigationResponse>() {
            @Override
            public void onSuccess(NavigationResponse response) {
                System.out.println("Got: " + response.toString());
            }

            @Override
            public void onFailure(RemoteException e) {

            }
        });
    }

    public void cancel() throws ServiceNotFoundException {
        ServiceClient<Object, EmptyResponse> client = node.newServiceClient(name + "/command/cancel", EmptyRequest._TYPE);
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

    public String getName() {
        return name;
    }

    public Observable<String> getLog() {
        return logSubject;
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
        logSubscriber.shutdown();
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
