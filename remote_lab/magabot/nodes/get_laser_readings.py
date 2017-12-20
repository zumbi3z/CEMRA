#! /usr/bin/env python

import rospy
import math
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry
import tf


odom = None


def save_odom(msg):
  global odom
  
  odom = msg


def save_laser(msg):
  if odom == None:
    yaw = 0.0
  else:
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x,
                     odom.pose.pose.orientation.y,
                     odom.pose.pose.orientation.z,
                     odom.pose.pose.orientation.w])

  laser_xy = PoseArray()
  max_x = 0.0
  max_y = 0.0
  #rospy.loginfo("YAW ---> %f" % yaw)
  for k in range(0, len(msg.ranges)):
    pose = Pose()
    if msg.ranges[k] != float('inf'):
      x = msg.ranges[k] * math.cos(k * msg.angle_increment + msg.angle_min)# + yaw)
      y = msg.ranges[k] * math.sin(k * msg.angle_increment + msg.angle_min)# + yaw)
      pose.position.x = x
      pose.position.y = y
      laser_xy.poses.append(pose)
      if x == float('inf') or y == float('inf'):
        rospy.loginfo("X %f", x)
        rospy.loginfo("Y %f", y)
        rospy.loginfo("YAW %f", yaw)
        rospy.loginfo("RANGE %f", msg.ranges[k])

      if math.fabs(x) > max_x:
        max_x = math.fabs(x)
      if math.fabs(y) > max_y:
        max_y = math.fabs(y)
  
  laser_xy.header.frame_id = str(max_x) + " " + str(max_y) + " " + str(yaw)
  pub.publish(laser_xy)
    
  
if __name__ == '__main__':
  rospy.init_node('laser_readings')
  
  pub = rospy.Publisher('laser_xy', PoseArray)
  rospy.Subscriber('scan', LaserScan, save_laser)
  rospy.Subscriber('odom', Odometry, save_odom)

  rospy.spin()
