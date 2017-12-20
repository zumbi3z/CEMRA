#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import * 
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from ca_msgs.msg import *
import threading
import subprocess
import os
import time



class autoDockingProcedure(object):

	def __init__(self):
		#Class constructor, will be executed at the moment of object creation
		#Subscribe to topic charge_rate
		self.subscriber = rospy.Subscriber('robot1/battery/charge_ratio', Float32, self.batteryCallback)
		self.publisher = rospy.Publisher('robot1/dock', Empty)
		self.publisher_2 = rospy.Publisher('robot1/undock', Empty)
		self.publisher_led = rospy.Publisher('robot1/dock_led', Bool)
		# get from parameter server the frequency at wich this node will run
		self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate',0.1))
		
		# declare member variables
		self.battery_msg = Float32()
		self.battery_msg.data = None
		self.batt_received = False
		self.event_in_msg_received = False
		# print message in terminal
		rospy.loginfo('autodocking initialized')

	def autodock(self):
		# this code evaluates send the robot to the docking station
		rospy.loginfo(" --- Robot going to charge --- ")
		self.publisher.publish(Empty())
		while self.battery_msg.data <=0.95:
			self.publisher_led.publish(True)
			rospy.loginfo(self.battery_msg.data)
			self.loop_rate.sleep()
			
	def undock(self):
		# this code sends the robot to the docking station
		rospy.loginfo(self.battery_msg.data)
		self.publisher.publish(Empty())
		#sleep(1)
		self.publisher_2.publish(Empty())
		
	def batteryCallback(self, msg):
		# this code executes every time you receive a message on this topic

		self.batt_received = True
		self.battery_msg = msg

	def start_node(self):
		while not rospy.is_shutdown():
			if self.batt_received:
				self.batt_received = False
				rospy.loginfo(self.battery_msg.data)
				# Evaluates the battery charge and sends the robot to charge --- It's needed to add a time counter!!
				if self.battery_msg.data < 0.30:
					self.autodock()	
				# if the battery is charged, sends it to work again
				elif self.battery_msg.data  >= 0.95:
					rospy.loginfo(" --- Robot charged --- ")
					self.undock()
			self.loop_rate.sleep()

if __name__=='__main__':
	# register node in ros network
	rospy.init_node('autodocking_procedure_test_node', anonymous=False)
	# create object of the class autoDockingProcedure (constructor will be executed)
	my_object = autoDockingProcedure()
	# call start_node method of class autoDockingProcedure
	my_object.start_node()