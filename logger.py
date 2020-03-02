#!/usr/bin/env python
import rospy
import led
import sched, time
import argparse
from datetime import datetime, timedelta
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

s = sched.scheduler(time.time, time.sleep)

position = None
positionReceived = None
co2Received = None

def validUpdate(inputTime):
    return inputTime is not None and datetime.now() - inputTime < timedelta(seconds = 3)

def updateStatus(position = None, co2 = None): 
    global positionReceived
    global co2Received
    if position is not None:
        positionReceived = datetime.now()
    if co2 is not None:
        co2Received = datetime.now()

def updateLED(sc):
    validPosition = validUpdate(positionReceived)
    validCo2 = validUpdate(co2Received)
    led.setColor([255 if validPosition and not validCo2 else 0,  
        255 if validPosition and validCo2 else 0,
        255 if not validPosition and validCo2 else 0])
    s.enter(1, 1, updateLED, (sc,))

def callback(data):
    global position
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    position = data
    updateStatus(position = True)

def co2Callback(data):
    global position
    updateStatus(co2 = True)
    if position is not None:
        print "{} co2: '{}' @ {} {} {}".format(datetime.now(), data, position.latitude, position.longitude, position.altitude)
    else:
        print "{} cos: '{}' @ -".format(datetime.now(), data)

def listener(id):

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gpslistener', anonymous=True)

    rospy.Subscriber("{}/mavros/global_position/global".format(id), NavSatFix, callback)
    rospy.Subscriber("{}/co2".format(id), String, co2Callback)

    s.enter(1, 1, updateLED, (s,))
    s.run()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = 'Log the given drone\'s GPS And CO2.')
    parser.add_argument('id', type=str, help='Name of the drone.')
    args = parser.parse_args()

    listener(args.id)
