#!/usr/bin/env python3
import rx
import rx.operators as ops
import numpy as np
from std_msgs.msg import String

from .ActionState import ActionState


class CalibrateAction:
    MAX_VELOCITY = 1.0
    SAMPLE_RATE = .01
    AVERAGE_TIME = 60

    def __init__(self, id, log_publisher, drones, droneStreamFactory):
        self.id = id
        self.log_publisher = log_publisher
        self.drones = set(drones)
        self.commanded = False
        self.status = ActionState.WORKING
        self.droneStreamFactory = droneStreamFactory

        self.gradient_subscription = rx.empty().subscribe()
        self.timerSubscription = rx.empty().subscribe()
        self.max_value = None

    def average(self, drone, data):

        self.log_publisher.publish(String(data="Average for {}: {}".format(drone.name, np.average(data))))
        self.log_publisher.publish(String(data="Stddev for {}: {}".format(drone.name, np.std(data))))

        drone.set_co2_statistics(np.average(data), np.std(data))

    def step(self):
        if not self.commanded:
            print("Calibrating")
            self.commanded = True

            for drone in self.drones:

                drone_stream = self.droneStreamFactory.get_drone(drone)

                drone_stream.get_co2().pipe(
                    ops.map(lambda reading: reading.ppm),
                    ops.buffer(timespan=self.AVERAGE_TIME)
                ).subscribe(
                    on_next=lambda data, drone=drone: self.average(drone, data))

        return ActionState.SUCCESS

    def stop(self):
        #self.gradient_subscription.dispose()
        pass
