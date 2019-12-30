#!/usr/bin/env python
import rospy
import serial
import time
from std_msgs.msg import String
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL

#this->arm_client = nodeHandle.serviceClient<mavros_msgs::CommandBool>(name + "/mavros/cmd/arming");
#this->takeoff_client = nodeHandle.serviceClient<mavros_msgs::CommandTOL>(name + "/mavros/cmd/takeoff");
#this->land_client = nodeHandle.serviceClient<mavros_msgs::CommandTOL>(name + "/mavros/cmd/land");

rospy.init_node('arm_test_service')
rospy.wait_for_service('JUAV1/mavros/set_mode')
rospy.wait_for_service('JUAV1/mavros/cmd/arming')
rospy.wait_for_service('JUAV1/mavros/cmd/takeoff')
rospy.wait_for_service('JUAV1/mavros/cmd/land')

setmode_service = rospy.ServiceProxy('JUAV1/mavros/set_mode', SetMode)
arm_service = rospy.ServiceProxy('JUAV1/mavros/cmd/arming', CommandBool)
takeoff_service = rospy.ServiceProxy('JUAV1/mavros/cmd/takeoff', CommandTOL)
land_service = rospy.ServiceProxy('JUAV1/mavros/cmd/land', CommandTOL)

print "Setup complete"

print "Set Mode"
print setmode_service(custom_mode = "GUIDED")	

time.sleep(10) 

print "Arming"
print arm_service(True)

time.sleep(10)
 
print "Disarming"
print arm_service(False)

print "Commanded"

