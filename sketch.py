#!/usr/bin/env python3
import math
import rx
import rx.operators as ops
import numpy as np
import matplotlib.pyplot as plt
from skimage import measure

from rx.subject import Subject, BehaviorSubject

EARTH_CIRCUMFERENCE = 40008000
FORWARD = "forward"
TURN = "turn"

class LatLon:

    def __init__(self, latitude, longitude):
        self.latitude = latitude
        self.longitude = longitude

class CO2:

    def __init__(self, ppm):
        self.ppm = ppm

class ReadingPosition:

    def __init__(self, latitude, longitude, value):
        self.latitude = latitude
        self.longitude = longitude
        self.value = value

class DroneStream:

    def __init__(self, name, position_subject, co2_subject):
        self.name = name

        self.mean = 0
        self.std_dev = 0
        self.position_subject = position_subject
        self.co2_subject = co2_subject

    def set_co2_statistics(self, mean, std_dev):
        self.mean = mean
        self.std_dev = std_dev

    def get_position(self):
        return self.position_subject

    def get_co2(self):
        return self.co2_subject

class DroneStreamFactory:

    def __init__(self):
        self.drones = {}

    def put_drone(self, name, position_subject, co2_subject):
        self.drones[name] = DroneStream(name, position_subject, co2_subject)

    def get_drone(self, name):
        if name not in self.drones.keys():
            self.drones[name] = DroneStream(name, self.node)

        return self.drones[name]

class PositionVector:
    a = 0

    def __init__(self):
        pass
        # self.movement = movement
        # self.position = position
        # self.x = x
        # self.y = y
        # self.distance = distance
        # self.radius = radius
        # self.center = center
        # self.a = a
        # self.p = p

def differenceInMeters(one, two):
    earthCircumference = 40008000
    return [
        ((one.longitude - two.longitude) * (earthCircumference / 360) * math.cos(one.latitude * 0.01745)),
        ((one.latitude - two.latitude) * (earthCircumference / 360))
    ]
VIRTUAL_SOURCE = LatLon(35.19465, -106.59625)

def calculateCO2xy(latitude, longitude):
    return calculateCO2(LatLon(latitude, longitude))

def calculateCO2(position):

    [y, x] = differenceInMeters(position, VIRTUAL_SOURCE)

    if x >= 0:
        return 420.0

    # Simple gaussian plume model adapted from: https://epubs.siam.org/doi/pdf/10.1137/10080991X
    # See equation 3.10, page 358.
    Q = 5000 # kg/s Emission Rate
    K = 2 # Diffusion Constant
    H = 2 # m Height
    u = 1 # m/s Wind Speed

    value = (Q / (2 * math.pi * K * -x)) * math.exp(- (u * ((pow(y, 2) + pow(H, 2))) / (4 * K * -x)))

    if value < 0:
        return 420.0
    else:
        return 420.0 + value

class SketchAction:
    SAMPLE_RATE = 0.1

    def __init__(self, id, local_setvelocity_publisher, announce_stream, offset, partner, leader,
                 drone_stream_factory, dragonfly_sketch_subject, position_vector_publisher):
        # self.log_publisher = log_publisher
        # self.logger = logger
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.id = id
        self.partner = partner
        self.offset = offset
        self.leader = leader
        self.started = False
        self.ros_subscriptions = []
        self.announce_stream = announce_stream
        self.drone_stream_factory = drone_stream_factory
        self.dragonfly_sketch_subject = dragonfly_sketch_subject
        self.position_vector_publisher = position_vector_publisher

        self.navigate_subscription = rx.empty().subscribe()
        self.leader_broadcast_subscrition = rx.empty().subscribe()
        self.target_position_vector = None
        self.encountered = False

    def navigate(self, input):

        twist = [0, 0]

        for vector in input:
            twist[0] += vector[0]
            twist[1] += vector[1]

        self.local_setvelocity_publisher.on_next(twist)

    def differenceInMeters(self, one, two):
        return [
            ((one.longitude - two.longitude) * (EARTH_CIRCUMFERENCE / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (EARTH_CIRCUMFERENCE / 360))
        ]

    def format_velocities(self, twist):
        return [
            twist.twist.linear.x,
            twist.twist.linear.y
        ]

    def magnitude(self, vector):
        return np.linalg.norm(vector)

    def step(self):
        if not self.started:
            self.started = True

            # self.logger.info("Subscribing...")

            partner_drone = self.drone_stream_factory.get_drone(self.partner)
            self_drone = self.drone_stream_factory.get_drone(self.id)

            self.navigate_subscription = rx.combine_latest(
                partner_drone.get_position(),
                self_drone.get_position(),
                self.dragonfly_sketch_subject
            ).pipe(
                # ops.sample(self.SAMPLE_RATE),
                ops.map(lambda positions: self.tandem(positions[0], positions[1], positions[2]))
            ).subscribe(lambda vectors: self.navigate(vectors))

            if self.leader:
                self.leader_broadcast_subscrition = rx.combine_latest(
                    self.setupSubject(self.partner),
                    self.setupSubject(self.id)
                ).subscribe(lambda positionReadingVector: self.broadcast_target(positionReadingVector[0], positionReadingVector[1]))

            # self.log_publisher.publish(String(data="Sketch"))

    def setupSubject(self, drone):

        drone_streams = self.drone_stream_factory.get_drone(drone)

        position_subject = drone_streams.get_position()
        co2_subject = drone_streams.get_co2()

        position_value_subject = Subject()

        rx.combine_latest(position_subject, co2_subject).pipe(
            ops.map(lambda tuple, offset=drone_streams.mean: ReadingPosition(tuple[0].latitude, tuple[0].longitude, tuple[1].ppm - offset))
        ).subscribe(on_next=lambda v: position_value_subject.on_next(v))

        return position_value_subject

    def tandem(self, partner_position, self_position, positionVector):

        tandem_offset = self.caculate_tandem_offset(partner_position, self_position)
        tandem_angle = self.calculate_tandem_angle(partner_position, self_position, positionVector)
        straight_line = self.calculate_straight_line(positionVector)
        turning = self.calculate_turn(partner_position, self_position, positionVector)
        offset_error_correction = self.calculate_error_correction(partner_position, self_position, positionVector)

        # print(f"{self.id} tandem_offset: {tandem_offset} tandem_angle: {tandem_angle} straight_line: {straight_line}")

        return [tandem_offset, tandem_angle, straight_line, turning, offset_error_correction]

    def caculate_tandem_offset(self, partner_position, self_position):
        distance_m = self.differenceInMeters(partner_position, self_position)
        offset_unitary = self.unitary(distance_m)

        possition_offset = [distance_m[0] - (offset_unitary[0] * self.offset),
                            distance_m[1] - (offset_unitary[1] * self.offset)]
        offset_magnitude = self.magnitude(possition_offset)

        tandem_distance = [
            2 * possition_offset[0] / max(2, offset_magnitude),
            2 * possition_offset[1] / max(2, offset_magnitude)
        ]
        return tandem_distance

    def calculate_tandem_angle(self, partner_position, self_position, positionVector):
        if positionVector.movement == FORWARD:
            distance_m = self.differenceInMeters(self_position, partner_position)
            offset_unitary = self.unitary(distance_m)

            direction_vector = [positionVector.x, positionVector.y]

            # print (offset_unitary)
            # print(direction_vector)

            return -2 * (np.dot(offset_unitary, direction_vector)) * np.array(direction_vector)
        else:
            return [0, 0]

            # average_position =  dotdict({'latitude':  (self_position.latitude + partner_position.latitude) / 2,
            #                              'longitude': (self_position.longitude + partner_position.longitude) / 2})
            #
            # distance_m = self.differenceInMeters(average_position, positionVector.center)
            # distance_unitary = self.unitary(distance_m);
            #
            # return [distance_unitary[1], distance_unitary[0]]

    def calculate_straight_line(self, positionVector):
        if positionVector.movement == FORWARD:
            return [3 * positionVector.x, 3 * positionVector.y]
        else:
            return [0, 0]

    def rotate_vector(self, vector, angle):
        rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                    [np.sin(angle), np.cos(angle)]])
        return np.dot(rotation_matrix, vector)


    def calculate_turn(self, partner_position, self_position, positionVector):
        if positionVector.movement == TURN:

            self_center = self.differenceInMeters(positionVector.center, self_position)
            self_center_distance = self.magnitude(self_center)
            partner_center_distance = self.magnitude(self.differenceInMeters(positionVector.center, partner_position))

            if self_center_distance > partner_center_distance:
                velocity = 3
            else:
                velocity = (self_center_distance / partner_center_distance) * 3

            vector_to_center = self.unitary(self_center)

            starting_position = self.differenceInMeters(positionVector.center, positionVector.position)
            # print(f"{self_center} {starting_position}")
            angle = math.atan2(self_center[1], self_center[0]) - math.atan2(starting_position[1], starting_position[0])

            rotated_vector = self.rotate_vector([positionVector.x, positionVector.y], angle)

            # print(f"Angle: {angle}, rotated_vector: {rotated_vector}")

            forward_force = np.array([rotated_vector[0] * velocity, rotated_vector[1] * velocity])
            centripedal_force = np.array([vector_to_center[0] * (velocity / self_center_distance), vector_to_center[1] * (velocity / self_center_distance)])

            # print(f"forward_force: {forward_force}, centripedal_force: {centripedal_force}")
            # print(f"hyp: {hyp} target_offset: {target_offset}")

            return forward_force + centripedal_force
        else:
            return [0, 0]

    def calculate_error_correction(self, partner_position, self_position, positionVector):
        if positionVector.movement == FORWARD:
            lat_ave = (self_position.latitude + partner_position.latitude) / 2
            lon_ave = (self_position.longitude + partner_position.longitude) / 2

            vector_to_target =  self.differenceInMeters(LatLon(lat_ave, lon_ave), positionVector.position)

            dot_product = np.dot([positionVector.x, positionVector.y], vector_to_target)

            vector_to_line = [
                vector_to_target[0] - (dot_product * positionVector.x),
                vector_to_target[1] - (dot_product * positionVector.y)
            ]

            magnitude = self.magnitude(vector_to_line)

            if magnitude == 0:
                return [0, 0]
            else:
                return vector_to_line / self.magnitude(vector_to_line)
        else:
            return [0, 0]

    def unitary(self, vector):
        magnitude = self.magnitude(vector)
        return [vector[0] / magnitude, vector[1] / magnitude]

    def average(self, one, two):
        return [(one.longitude + two.longitude)/2, (one.latitude + two.latitude) / 2]

    def broadcast_target(self, partner_position, self_position):

        average_position =  LatLon((self_position.latitude + partner_position.latitude) / 2, (self_position.longitude + partner_position.longitude) / 2)

        if not self.target_position_vector:
            positionVector = PositionVector()
            positionVector.movement = FORWARD
            position = LatLon(average_position.longitude, average_position.latitude)

            positionVector.x = 0.0
            positionVector.y = 1.0
            positionVector.position = position
            positionVector.distance = 10.0

            self.target_position_vector = positionVector

        if not self.encountered and (self.inside(partner_position) or self.inside(self_position)):
            self.encountered = True
            # print(f"Encountered plume: {partner_position.value} {self_position.value}")

        if self.encountered and self.passed(average_position):
            # print(f"PASSED! {positionVector} | {self.target_position_vector}")
            self.target_position_vector = self.boundary_sketch(partner_position, self_position)
        self.position_vector_publisher.on_next(self.target_position_vector)
        self.dragonfly_sketch_subject.on_next(self.target_position_vector)

    # Algorithm 1 Ensures the robots are at distance √
    #
    # λ from each other and are oriented in the same direction.
    # Additional discussion with illustrative diagrams of this synchronization is in Appendix D.
    #
    # procedure SYNCHRONIZE(D1, D2)
    #   Path ← the polyline path of D2 from last crossing of BOUNDARY-SKETCH with the shape till current position.
    #   ∇ ← the gradient at the last boundary crossing for D2.
    #   L1 ← the line in the direction of ∇ through D1’s position.
    #   L2 ← the line in the direction of ∇ through D2’s position.
    #   if L1 crosses Path then
    #       Move D2 in its current direction until it is √λ distance away from L1.
    #       Change direction to ∇ and take a single step of length λ.
    #       Move D1 along L1 until it is √λ away from D2.
    #
    #   else
    #       Move D1 in its current direction until it is √ λ distance away from L2.
    #
    #   Change direction to ∇ and move until the distance from D2 is √λ.

    # Algorithm 2 Reestablishes “Sandwich” Invariant
    # procedure CROSS-BOUNDARY(D1, D2, α)
    #   p ← last position of D1 before crossing
    #   R ← the vertices of the regular polygon including D1’s position with exterior angle √λ and
    #       the edge beginning at D1’s position facing the direction of ∇ + α.
    #   P ← the vertices of the convex hull of R ∪ {p}. For all i : 0 ≤ i ≤ |P| − 1, let
    #       Pi be the i-th vertex in this convex hull, ordered such that P0 = p and P1 = D1’s current position.
    #   ∇ ← gradient at the last boundary crossing of D1
    #   i ← 1.
    #   while neither robot has crossed the boundary AND i + 1 < |P| do
    #       D1 moves to Pi+1.
    #       D2 moves to closest point from it that is √λ distance away from Pi and orthogonal to ∇ + iα
    #       i ← i + 1
    #   while neither robot has crossed the boundary do
    #       D1 moves towards point p taking steps of length λ.
    #       D2 moves to closest point from it that is √λ distance away from D1 and orthogonal to D1’s direction.
    #   if D2 crossed the boundary then
    #       SYNCHRONIZE (D1, D2)
    #   else
    #       ∇ ← the current direction of D1.



    def cross_boundary(self, d1, d2, a):
        return self.turn(d1, d2, a)

    # Algorithm 3 Initially, robots are √λ apart; one inside and one outside
    def boundary_sketch(self, d1, d2):
        # D1, D2 ← the two robots
        # ∇ ← boundary gradient at point of crossing with line segment between D1 and D2
        # α ← √λ
        lambda_value = 0.01
        if self.inside(d1) ^ self.inside(d2):
            # D1 and D2 both move λ distance in the direction of ∇
            print("Sandwich")
            return self.forward(d1, d2)

        if not self.inside(d1) and not self.inside(d2):
            print("Outside")
            a = -math.sqrt(lambda_value)
            return self.cross_boundary(d1, d2, a)
        elif self.inside(d1) and self.inside(d2):
            print("Inside")
            a = math.sqrt(lambda_value)
            return self.cross_boundary(d1, d2, a)

    def calculate_direction(self):
        if self.target_position_vector.movement == FORWARD:
            direction = [self.target_position_vector.x, self.target_position_vector.y]
            print(f"forward: {direction}")
            return direction
        if self.target_position_vector.movement == TURN:
            prev_vector = [self.target_position_vector.x, self.target_position_vector.y]
            angle =  self.target_position_vector.a * self.target_position_vector.p
            direction = self.rotate_vector(prev_vector, angle)
            print(f"turn {self.target_position_vector.a} * {self.target_position_vector.p} = {angle * 57.2958}: {direction} {self.rotate_vector(prev_vector, angle)}")
            return direction

    def forward(self, d1, d2):
        average_position = self.average(d1, d2)

        positionVector = PositionVector()
        positionVector.movement = FORWARD
        position = LatLon(average_position[0], average_position[1])
        positionVector.position = position
        [positionVector.x, positionVector.y] = self.calculate_direction()
        positionVector.position = position
        positionVector.distance = 10.0

        return positionVector

    def turn(self, d1, d2, a):
        if a == self.target_position_vector.a:
            self.target_position_vector.p += 1
            print(f"p : {self.target_position_vector.p}")
            return self.target_position_vector
        average_position = self.average(d1, d2)
        distance_m = self.differenceInMeters(d1, d2)
        offset_unitary = self.unitary(distance_m)

        positionVector = PositionVector()
        positionVector.movement = TURN

        position = LatLon(average_position[1], average_position[0])
        positionVector.position = position
        [positionVector.x, positionVector.y] = self.calculate_direction()

        positionVector.position = position
        positionVector.radius = 10.0
        positionVector.a = a
        positionVector.p = 1

        hyp = 1 / np.arcsin(a)
        # print(hyp)


        longitude = average_position[0] + ((offset_unitary[0] * hyp) / (math.cos(average_position[1] * 0.01745) * (EARTH_CIRCUMFERENCE / 360)))
        latitude = average_position[1] - ((offset_unitary[1] * hyp) / (EARTH_CIRCUMFERENCE / 360))
        center = LatLon(latitude, longitude)

        # print(f"center lon: {center.longitude} lat: {center.latitude}")

        positionVector.center = center

        return positionVector

    def inside(self, d):
        return d.value > 421

    def passed(self, average_position):
        if self.target_position_vector is None:
            return True

        if self.target_position_vector.movement == FORWARD:

            target_offset = self.differenceInMeters(average_position, self.target_position_vector.position)

            return np.dot(target_offset, [self.target_position_vector.x, self.target_position_vector.y]) > self.target_position_vector.distance
        else:
            target_offset = self.differenceInMeters(self.target_position_vector.position, self.target_position_vector.center)
            hyp = self.differenceInMeters(average_position, self.target_position_vector.center)

            # print(f"({target_offset} / {hyp}) > math.sin({self.target_position_vector.a} * {self.target_position_vector.p}) = {(target_offset / hyp) > math.sin(self.target_position_vector.a * self.target_position_vector.p)}")
            # return (target_offset / hyp) > math.sin(self.target_position_vector.a * self.target_position_vector.p)
            # return False

            target_angle = math.fabs(self.target_position_vector.a * self.target_position_vector.p)

            angle = math.acos( ((target_offset[0] * hyp[0]) + (target_offset[1] * hyp[1])) / (self.magnitude(target_offset) * self.magnitude(hyp)))

            # self.logger.info(f"Angle: {angle} passed: {angle} > {target_angle} = {angle > target_angle}")
            return angle > target_angle


    def stop(self):
        self.navigate_subscription.dispose()
        self.leader_broadcast_subscrition.dispose()

def gaussians(x, y, gaussian_list):
    value = 0
    for gaussian in gaussian_list:
        value = value + gaussian_value(x, y, gaussian)
    return value

def gaussian_value(x, y, gaussian):
    xoffset = x - gaussian[0][0]
    yoffset = y - gaussian[0][1]
    varX = gaussian[1][0]
    varY = gaussian[1][1]
    return np.exp ( - (xoffset * xoffset / (2*varX)) - (yoffset * yoffset / (2*varY)))

def plot_plume(plt):
    ax = plt.gca()

    scale = 1000.0

    lon = [-106.6, -106.585]
    lat = [35.190, 35.220]

    lonRes = ((lon[1] - lon[0]) / scale)
    latRes = ((lat[1] - lat[0]) / scale)
    x, y = np.ogrid[lon[0]:lon[1]:lonRes, lat[0]:lat[1]:latRes]

    calculateCO2xyvect = np.vectorize(calculateCO2xy)

    r = calculateCO2xyvect(y, x)

    contour = measure.find_contours(r, 421)

    ax.plot(contour[0][:, 0] * lonRes + lon[0], contour[0][:, 1] * latRes + lat[0], linewidth=3, color="lightgreen",zorder=-1, label="Plume boundary")

def main():

    streamFactory = DroneStreamFactory()
    announceStream = Subject()
    df1SetVelocity = BehaviorSubject([0,1])
    df2SetVelocity = BehaviorSubject([0,1])
    sketchSubject = Subject()
    vector1Publisher = Subject()
    vector2Publisher = Subject()
    df1Position = Subject()
    df2Position = Subject()
    df1co2 = Subject()
    df2co2 = Subject()
    streamFactory.put_drone("DF1", df1Position, df1co2)
    streamFactory.put_drone("DF2", df2Position, df2co2)

    action1 = SketchAction("DF1", df1SetVelocity, announceStream, 10, "DF2", True, streamFactory, sketchSubject, vector1Publisher)
    action2 = SketchAction("DF2", df2SetVelocity, announceStream, 10, "DF1", False, streamFactory, sketchSubject, vector2Publisher)

    action1.step()
    action2.step()

    # sketchSubject.on_next(dotdict({'movement': FORWARD}))
    # sketchSubject.subscribe(on_next = lambda value: print(value))

    df1p = LatLon(35.191558837890625, -106.59635925292969)
    df2p = LatLon(35.19157791137695, -106.59661102294922)

    def addOffset(position, offset):
        longitude = position.longitude + (offset[0] / (math.cos(position.latitude * 0.01745) * (EARTH_CIRCUMFERENCE / 360)))
        latitude = position.latitude + (offset[1] / (EARTH_CIRCUMFERENCE / 360))
        return LatLon(latitude, longitude)

    df1latitudes = []
    df2latitudes = []
    df1longitudes = []
    df2longitudes = []

    def addAlgorithmDetails(token):
        if (token.movement == FORWARD):
            pass
        if (token.movement == TURN):
            plt.scatter(token.center.longitude, token.center.latitude, color='b', s=3)

    sketchSubject.subscribe(on_next = lambda value: addAlgorithmDetails(value))


    plt.figure(figsize=(6, 10))
    plt.ticklabel_format(style='plain', useOffset=False)

    plot_plume(plt)

    for i in range(160):
        # print(i)
        df1co2v = calculateCO2(df1p)
        df2co2v = calculateCO2(df2p)
        df1Position.on_next(df1p)
        df1co2.on_next(CO2(df1co2v))
        df2Position.on_next(df2p)
        df2co2.on_next(CO2(df2co2v))

        df1velocity = df1SetVelocity.pipe(ops.first()).run()
        df2velocity = df2SetVelocity.pipe(ops.first()).run()

        df1p = addOffset(df1p, df1velocity)
        df2p = addOffset(df2p, df2velocity)


        # print(f"df1 co2: {df1co2v}")
        # print(f"df2 co2: {df2co2v}")
        # print(f"df1 v: {df1velocity}")
        # print(f"df2 v: {df2velocity}")
        #
        # print(f"df1 p: {df1p.latitude}, {df1p.longitude}")
        # print(f"df2 p: {df2p.latitude}, {df2p.longitude}")
        #
        # print()

        df1latitudes.append(df1p.latitude)
        df1longitudes.append(df1p.longitude)
        df2latitudes.append(df2p.latitude)
        df2longitudes.append(df2p.longitude)

    plt.scatter(df1longitudes, df1latitudes, s=4)
    plt.scatter(df2longitudes, df2latitudes, s=4)
    plt.axis('equal')
    plt.savefig('sketch.png', format="png", dpi=300)



if __name__ == "__main__":
    main()