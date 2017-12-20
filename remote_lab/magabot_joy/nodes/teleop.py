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

class teleOperation(object):
	def __init__(self):
		rospy.Subscriber('robot1/teleop' , String, self.robotCallback_1)
		rospy.Subscriber('robot2/teleop' , String, self.robotCallback_2)
		rospy.Subscriber('robot3/teleop' , String, self.robotCallback_3)
		self.pub_vel_1 = rospy.Publisher('robot1/cmd_vel', Twist)
		self.pub_vel_2 = rospy.Publisher('robot2/cmd_vel', Twist)
		self.pub_vel_3 = rospy.Publisher('robot3/cmd_vel', Twist)
		self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate',10))
		self.command = Twist()
		self.timeNow = None
		self.last_cmd = None
		self.last_cmd_time = None
		self.timeDiff = None
		self.isStopped = True
		self.isRestarted = False
		self.topics_to_record = None
		self.robot_name = 0
		self.robot_msg = String()
		self.robot_received = False
		self.def_lvel = 0.3
		self.def_avel = 0.3
		self.max_vel = 1.0
		self.min_vel = 0.1
		self.max_ang = math.pi / 2.0
		self.min_ang = 0.3
		self.secStop = 3.0
		self.secRestart = 30.0
		self.cmd_dict = None
		self.timeNow = None
		# dict de correspondencia entre STRING recebida e o comando a enviar ao robot
		self.cmd_dict = {'UP': self.up, 'DOWN': self.down, 'LEFT': self.left, 'RIGHT': self.right,
		    'VEL_UP': self.velUp, 'VEL_DOWN': self.velDown, 'ANG_UP': self.angUp, 'ANG_DOWN': self.angDown, 'STOP': self.stop}


		rospy.loginfo(' teleop initialized ')

	def robotCallback_1(self, msg):
	 	rospy.loginfo(' robot1 ')
	 	# if self.robot_received:
		self.robot_received = True
		self.robot_msg = msg
		self.robot_name = 'r1' 	
		self.teleop_interpreter()
		

	def robotCallback_2(self, msg):
	 	# if self.robot_received:
		self.robot_received = True
		self.robot_msg = msg
		rospy.loginfo(' robot2 ')
		self.robot_name = 'r2'
		self.teleop_interpreter()
		

	def robotCallback_3(self, msg):
		self.robot_received = True
		self.robot_msg = msg
		rospy.loginfo(' robot3 ')
		self.robot_name = 'r3'
		self.teleop_interpreter()
		
		 	

	
	def up(self):
		self.command.linear.x = self.lin_vel
		self.command.angular.z = 0.0
		rospy.loginfo("--- Going up ---")

	def down(self):
		self.command.linear.x = -self.lin_vel
		self.command.angular.z = 0.0
		rospy.loginfo("--- Going down ---")

	def left(self):
		self.command.linear.x = 0.0
		self.command.angular.z = -self.ang_vel
		rospy.loginfo(" --- Turning left --- ")

	def right(self):
		self.command.linear.x = 0.0
		self.command.angular.z = self.ang_vel
		rospy.loginfo(" --- Turning right --- ")

	def velUp(self):
		if self.command < self.max_vel-0.1:
			rospy.loginfo(" --- Increasing linear velocity --- ")
			self.command += 0.1
		else:
			rospy.loginfo(" --- Maximum linear velocity reached ---")

	def velDown(self):
		if self.command > self.min_vel:
			rospy.loginfo(" --- Decreasing linear velocity --- ")
			self.command -= 0.1
		else:
			rospy.loginfo(" --- Minimum linear velocity reached ---")

	def angUp(self):
		if self.ang_vel < self.max_ang:
			rospy.loginfo(" --- Increasing angular velocity --- ")
			self.ang_vel += 0.2
		else:
			rospy.loginfo(" --- Maximum angular velocity reached ---")


	def angDown(self):
		if self.ang_vel < self.min_ang:
			rospy.loginfo(" --- Decreasing angular velocity --- ")
			self.ang_vel -= 0.2
		else:
			rospy.loginfo(" --- Minimum angular velocity reached ---")


	def stop(self):
		self.command.linear.x = 0.0
		self.command.angular.z = 0.0
		self.pub_vel_1.publish(self.command)
		self.pub_vel_2.publish(self.command)
		self.pub_vel_3.publish(self.command)
		rospy.loginfo(" --- Stopping --- ")


	def teleop_interpreter(self):
		
		print self.robot_msg 

		self.timeNow = rospy.Time.now()	
		if str(self.robot_msg.data) in self.cmd_dict.keys():
			self.cmd_dict[self.robot_msg.data]()
			self.last_cmd_time = self.timeNow
			# Now has to publish the command for one specific robot on the list
			
			if self.robot_received:
				self.robot_received = False
				if self.robot_name == 'r1':
					self.pub_vel_1.publish(self.command)
					rospy.loginfo('moving robot1')
				elif self.robot_name == 'r2':
					self.pub_vel_2.publish(self.command)
					rospy.loginfo('moving robot2')
				elif self.robot_name == 'r3':
					self.pub_vel_3.publish(self.command)
					rospy.loginfo('moving robot3')
			

			elif str(self.robot_msg.data) not in ["STOP", "VEL_UP", "VEL_DOWN", "ANG_UP", "ANG_DOWN"]:
				self.isStopped = False
				self.isRestarted = False

				print self.command
			else:
				self.isStopped = True
		else:
			rospy.logerr("The command is not valid. Check the documentation for the valid commands.")

		self.loop_rate.sleep()

	
	# def robotSelector(self):
	# 	if self.robot_name == 'robot1':
	# 			self.pub_vel_1.publish(self.command)
	# 	elif self.robot_name == 'robot2':
	# 			self.pub_vel_2.publish(self.command)
	# 	else self.robot_name == 'robot3':
	# 			self.pub_vel_3.publish(self.command)

	def compute_timeDiff(self):
		
		while not rospy.is_shutdown():
			self.timeNow = rospy.Time.now()

			if self.last_cmd_time != None:
				self.timeDiff = self.timeNow.secs - self.last_cmd_time.secs

				if not self.isStopped and (self.timeDiff >= self.secStop and self.timeDiff < self.secRestart):
					self.stop()
					self.isStopped = True

				elif self.isStopped and not self.isRestarted and self.timeDiff >= self.secRestart:
					self.stop()
					self.lin_vel = self.def_lvel
					self.ang_vel = self.def_avel
					self.isRestarted = True
					self.last_cmd_time = None
					rospy.loginfo(" --- Robot restarted --- ")

		self.loop_rate.sleep()

	def initialize(self):
	
		rospy.loginfo("Initializing teleop")
	
		self.lin_vel = self.def_lvel
		self.ang_vel = self.def_avel

		time.sleep(1)
		

if __name__ == '__main__':

	# register node in ros network
	rospy.init_node('robot_teleop', anonymous=False)
	# create object of the class teleOperation (constructor will be executed)
	my_object = teleOperation()
	# call start_node method of class teleOperation

	my_object.initialize()

	my_object.compute_timeDiff()

	rospy.spin()
