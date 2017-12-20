#! /usr/bin/env python

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String
import threading
import math
import subprocess
import os
from magabot.msg import Topics
import time

class runTimeTopicExample(object):

	def __init__(self):
		self.subscriber=rospy.Subscriber('robot1/teleop', String, self.callback_1)
		self.subscriber=rospy.Subscriber('robot2/teleop', String, self.callback_2)
		self.subscriber=rospy.Subscriber('robot3/teleop', String, self.callback_3)
		self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 1.0))
		self.test_msg = None
		self.twist_msg = String()
		self.twist_msg.data = 'initial content'
		self.twist_received = False
		self.event_in_msg_received = False
		self.robot_name = 'none'

	def callback_1(self, msg):
		self.twist_received = True
		self.twist_msg = msg
		self.robot_name = 'R1'
		rospy.loginfo("robot1 listening", msg)

	def callback_2(self, msg):
		self.twist_received = True
		self.twist_msg = msg
		self.robot_name = 'R2'
		rospy.loginfo("robot2 listening", msg)

	def callback_3(self, msg):
		self.twist_received = True
		self.twist_msg = msg
		self.robot_name = 'R3'
		rospy.loginfo("robot3 listening", msg)

	def robot1(self):
		rospy.loginfo(' moviendo el 1')

	def robot2(self):
		rospy.loginfo(' moviendo el 1')

	def robot3(self):
		rospy.loginfo(' moviendo el 1')

	def start_node(self):
		while not rospy.is_shutdown():
			if self.twist_received:
				self.twist_received = False
				rospy.loginfo(self.twist_msg.data)
				#Check if a request was received
			if self.event_in_msg_received:
				self.event_in_msg_received = False
				if self.robot_name == 'R1':
					self.robot1()
				elif self.robot_name == 'R2':
					self.robot2()
				elif self.robot_name == 'R3':
					self.robot3()
			else:
				rospy.loginfo(' no msg in topics ')

			self.loop_rate.sleep()

if __name__=='__main__':
	rospy.init_node('runtime_topic_example', anonymous=False)
	my_object = runTimeTopicExample()
	my_object.start_node()



 