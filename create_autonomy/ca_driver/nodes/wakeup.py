#! /usr/bin/env python

import rospy
import threading
import subprocess
import os
from std_msgs.msg import String
from std_msgs.msg import Empty


class wakeupProcedure(object):

	def __init__(self):

		self.wake_sub = rospy.Subscriber('/wakeup', String, robotnameCallback)
		self.pub_wake_1 = rospy.Publisher('robot1/undock', Empty)
		self.pub_wake_2 = rospy.Publisher('robot2/undock', Empty)
		self.pub_wake_3 = rospy.Publisher('robot3/undock', Empty)
		#get robot name from remote lab
		self.robot_received = String()
		self.robot_name = None
		self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate',1))

	def robotnameCallback(self, msg):

		self.robot_received = True
		self.robot_name = msg
		if self.robot_name == 'robot1':
			self.pub_wake_1.publish(Empty())	
		elif self.robot_name == 'robot2':
			self.pub_wake_2.publish(Empty())
		elif self.robot_name == 'robot3':
			self.pub_wake_3.publish(Empty())

	def start_node(self):

		while not rospy.is_shutdown():
			if self.robot_received:
				self.robot_received = False
	
			self.loop_rate.sleep()

if __name__=='__main__':
	# register node in ros network
	rospy.init_node('wakeup_procedure_test_node', anonymous=False)
	# create object of the class wakeupProcedure (constructor will be executed)
	my_object = wakeupProcedure()
	# call start_node method of class wakeupProcedure
	my_object.start_node()