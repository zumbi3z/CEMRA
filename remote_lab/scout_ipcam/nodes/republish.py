#! /usr/bin/env python

import rospy
from sensor_msgs.msg import *

def sub_callback(msg):
    global pub
    
    pub.publish(msg)
    
 
def sub_callback2(msg):
    global pub2
    
    pub2.publish(msg)
    

if __name__=="__main__":

    global pub, pub2

    rospy.init_node("republisher")
    pub = rospy.Publisher("/camera1/image_raw", Image)
    pub2 = rospy.Publisher("/camera2/image_raw", Image)
    rospy.Subscriber("/camera_driver_91_1/image", Image, sub_callback)
    rospy.Subscriber("/camera_driver_92_1/image", Image, sub_callback2)

    rospy.spin()
