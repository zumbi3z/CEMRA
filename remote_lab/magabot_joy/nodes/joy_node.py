#! /usr/bin/env python

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *


pub_vel = None


def process_joy_commands(msg):
  
  left_joystick = [msg.axes[0], msg.axes[1]]
  right_joystick = [msg.axes[2], msg.axes[3]]
  
  command = Twist()
  command.linear.x = left_joystick[1]
  command.angular.z = right_joystick[0]
  pub_vel.publish(command)


if __name__ == '__main__':
  global pub_vel
  
  rospy.init_node('joy_node_test')
  rospy.Subscriber('/joy', Joy, process_joy_commands)
  pub_vel = rospy.Publisher('cmd_vel', Twist)
  
  rospy.spin()






