import rospy
from datetime import datetime
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix

position = None

def callback(data):
    global position
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    position = data

def co2Callback(data):
    global position
    if position is not None:
        print "{} co2: '{}' @ {} {} {}".format(datetime.now(), data, position.latitude, position.longitude, position.altitude)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('gpslistener', anonymous=True)

    rospy.Subscriber('/mavros/global_position/global', NavSatFix, callback)
    rospy.Subscriber('JUAV1/co2', String, co2Callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

