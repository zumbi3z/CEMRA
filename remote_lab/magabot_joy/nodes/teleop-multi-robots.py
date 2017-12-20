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

# global variables
command = Twist()

last_cmd = None
last_cmd_time = None
timeDiff = None

isStopped = True
isRestarted = False

topics_to_record = None


def up():
	global command
	command.linear.x = lin_vel
	command.angular.z = 0.0
	rospy.loginfo("--- Going up ---")
def down():
	global command
	command.linear.x = -lin_vel
	command.angular.z = 0.0
	rospy.loginfo("--- Going down ---")
def left():
	global command
	command.linear.x = 0.0
	command.angular.z = ang_vel
	rospy.loginfo(" --- Turning left --- ")
def right():
	global command
	command.linear.x = 0.0
	command.angular.z = -ang_vel
	rospy.loginfo(" --- Turning right --- ")

def velUp():
	global lin_vel
	if lin_vel < max_vel-0.1:
		rospy.loginfo(" --- Increasing linear velocity --- ")
		lin_vel += 0.1
	else:
		rospy.loginfo(" --- Maximum linear velocity reached ---")

def velDown():
	global lin_vel
	if lin_vel > min_vel:
		rospy.loginfo(" --- Decreasing linear velocity --- ")
		lin_vel -= 0.1
	else:
		rospy.loginfo(" --- Minimum linear velocity reached ---")

def angUp():
	global ang_vel
	if ang_vel < max_ang:
		rospy.loginfo(" --- Increasing angular velocity --- ")
		ang_vel += 0.2
	else:
		rospy.loginfo(" --- Maximum angular velocity reached ---")


def angDown():
	global ang_vel
	if ang_vel < min_ang:
		rospy.loginfo(" --- Decreasing angular velocity --- ")
		ang_vel -= 0.2
	else:
		rospy.loginfo(" --- Minimum angular velocity reached ---")


def stop():
	global command
	command.linear.x = 0.0
	command.angular.z = 0.0
	pub_vel.publish(command)
	rospy.loginfo(" --- Stopping --- ")

# dict de correspondencia entre STRING recebida e o comando a enviar ao robot
cmd_dict = {'UP': up, 'DOWN': down, 'LEFT': left, 'RIGHT': right,
	    'VEL_UP': velUp, 'VEL_DOWN': velDown, 'ANG_UP': angUp, 'ANG_DOWN': angDown, 'STOP': stop}


def teleop_interpreter(msg):
	global lin_vel, ang_vel, last_cmd, last_cmd_time, isStopped, isRestarted

	print msg
	r = rospy.Rate(10)
	timeNow = rospy.Time.now()
	if str(msg.data) in cmd_dict.keys():
		cmd_dict[msg.data]()
		last_cmd_time = timeNow

		#if(robot_name == robot name in telop command)
		pub_vel.publish(command)

		if str(msg.data) not in ["STOP", "VEL_UP", "VEL_DOWN", "ANG_UP", "ANG_DOWN"]:
			isStopped = False
			isRestarted = False

			print command
		else:
			isStopped = True
	else:
		rospy.logerr("The command is not valid. Check the documentation for the valid commands.")
	r.sleep()

def compute_timeDiff():
	global timeDiff, last_cmd_time, last_cmd, lin_vel, ang_vel, isRestarted, isStopped

	while not rospy.is_shutdown():
		timeNow = rospy.Time.now()

		if last_cmd_time != None:
			timeDiff = timeNow.secs - last_cmd_time.secs

			if not isStopped and (timeDiff >= secStop and timeDiff < secRestart):
				stop()
				isStopped = True

			elif isStopped and not isRestarted and timeDiff >= secRestart:
				stop()
				lin_vel = def_lvel
				ang_vel = def_avel
				isRestarted = True
				last_cmd_time = None
				rospy.loginfo(" --- Robot restarted --- ")


# def manage_action(msg):

# 	print "MSG", msg

# 	if str(msg.data) == "stop_teleop" or str(msg.data) == "stop_all":
# 		rospy.loginfo("Killing teleoperation...")
# #		stop_rosbag()
# 		rospy.signal_shutdown("Killing teleoperation...")


# -- Stop rosbag cleanly --
#def stop_rosbag():
#	list_cmd = subprocess.Popen("rosnode list", shell = True, stdout = subprocess.PIPE)
#	list_output = list_cmd.stdout.read()
#	retcode = list_cmd.wait()
#	assert retcode == 0, rospy.logwarn("List command returned %d" % retcode)
#	for s in list_output.split("\n"):
#	    if (s.startswith("/record")):
#		os.system("rosnode kill " + s)


#def store_topic_names(msg):
#	global topics_to_record
#	print msg
#	topics_to_record = ''
#	for topic in msg.topics:
#		topics_to_record += topic + ' '

def initialize():
	global pub_vel, def_lvel, def_avel, max_vel, min_vel, max_ang, min_ang, lin_vel, ang_vel, secStop, secRestart, rosbag_proc

	rospy.init_node('robot_teleop')
	rospy.loginfo("Initializing...")

	#get robot name from remote lab
	#robot_name = rospy.get_name()
	#robot_name = robot_name.split("/");
	#robot_name = robot_name[1]
	#print robot_name


	#rospy.Subscriber('/topics_to_record', Topics, store_topic_names)
	rospy.Subscriber('robot1/teleop', String, teleop_interpreter)
	#rospy.Subscriber('/action', String, manage_action)
	pub_vel = rospy.Publisher('robot1/cmd_vel', Twist)

	if rospy.has_param("robot_teleop/def_lvel"):
		def_lvel = rospy.get_param("robot_teleop/def_lvel")
	else:
		def_lvel = 0.3

	if rospy.has_param("robot_teleop/def_avel"):
		def_avel = rospy.get_param("robot_teleop/def_avel")
	else:
		def_avel = 0.3

	if rospy.has_param("robot_teleop/max_vel"):
		max_vel = rospy.get_param("robot_teleop/max_vel")
	else:
		max_vel = 1.0

	if rospy.has_param("robot_teleop/min_vel"):
		min_vel = rospy.get_param("robot_teleop/min_vel")
	else:
		min_vel = 0.1

	if rospy.has_param("robot_teleop/max_ang"):
		max_ang = rospy.get_param("robot_teleop/max_vel")
	else:
		max_ang = math.pi / 2.0

	if rospy.has_param("robot_teleop/min_ang"):
		min_ang = rospy.get_param("robot_teleop/min_vel")
	else:
		min_ang = 0.3

	if rospy.has_param("robot_teleop/secStop"):
		secStop = rospy.get_param("robot_teleop/secStop")
	else:
		secStop = 3.0

	if rospy.has_param("robot_teleop/secRestart"):
		secRestart = rospy.get_param("robot_teleop/secRestart")
	else:
		secRestart = 30.0

	lin_vel = def_lvel
	ang_vel = def_avel

	time.sleep(1) # this sleep is just to ensure that the subscriber for the topics gets correctly initialized

	# # Robot_2

	# #rospy.Subscriber('/topics_to_record', Topics, store_topic_names)
	# rospy.Subscriber('robot2/teleop', String, teleop_interpreter)
	# #rospy.Subscriber('/action', String, manage_action)
	# pub_vel = rospy.Publisher('robot2/cmd_vel', Twist)

	# if rospy.has_param("robot_teleop/def_lvel"):
	# 	def_lvel = rospy.get_param("robot_teleop/def_lvel")
	# else:
	# 	def_lvel = 0.3

	# if rospy.has_param("robot_teleop/def_avel"):
	# 	def_avel = rospy.get_param("robot_teleop/def_avel")
	# else:
	# 	def_avel = 0.3

	# if rospy.has_param("robot_teleop/max_vel"):
	# 	max_vel = rospy.get_param("robot_teleop/max_vel")
	# else:
	# 	max_vel = 1.0

	# if rospy.has_param("robot_teleop/min_vel"):
	# 	min_vel = rospy.get_param("robot_teleop/min_vel")
	# else:
	# 	min_vel = 0.1

	# if rospy.has_param("robot_teleop/max_ang"):
	# 	max_ang = rospy.get_param("robot_teleop/max_vel")
	# else:
	# 	max_ang = math.pi / 2.0

	# if rospy.has_param("robot_teleop/min_ang"):
	# 	min_ang = rospy.get_param("robot_teleop/min_vel")
	# else:
	# 	min_ang = 0.3

	# if rospy.has_param("robot_teleop/secStop"):
	# 	secStop = rospy.get_param("robot_teleop/secStop")
	# else:
	# 	secStop = 3.0

	# if rospy.has_param("robot_teleop/secRestart"):
	# 	secRestart = rospy.get_param("robot_teleop/secRestart")
	# else:
	# 	secRestart = 30.0

	# lin_vel = def_lvel
	# ang_vel = def_avel

	# time.sleep(1) # this sleep is just to ensure that the subscriber for the topics gets correctly initialized

	# # Robot 3

	# #rospy.Subscriber('/topics_to_record', Topics, store_topic_names)
	# rospy.Subscriber('robot3/teleop', String, teleop_interpreter)
	# #rospy.Subscriber('/action', String, manage_action)
	# pub_vel = rospy.Publisher('robot3/cmd_vel', Twist)

	# if rospy.has_param("robot_teleop/def_lvel"):
	# 	def_lvel = rospy.get_param("robot_teleop/def_lvel")
	# else:
	# 	def_lvel = 0.3

	# if rospy.has_param("robot_teleop/def_avel"):
	# 	def_avel = rospy.get_param("robot_teleop/def_avel")
	# else:
	# 	def_avel = 0.3

	# if rospy.has_param("robot_teleop/max_vel"):
	# 	max_vel = rospy.get_param("robot_teleop/max_vel")
	# else:
	# 	max_vel = 1.0

	# if rospy.has_param("robot_teleop/min_vel"):
	# 	min_vel = rospy.get_param("robot_teleop/min_vel")
	# else:
	# 	min_vel = 0.1

	# if rospy.has_param("robot_teleop/max_ang"):
	# 	max_ang = rospy.get_param("robot_teleop/max_vel")
	# else:
	# 	max_ang = math.pi / 2.0

	# if rospy.has_param("robot_teleop/min_ang"):
	# 	min_ang = rospy.get_param("robot_teleop/min_vel")
	# else:
	# 	min_ang = 0.3

	# if rospy.has_param("robot_teleop/secStop"):
	# 	secStop = rospy.get_param("robot_teleop/secStop")
	# else:
	# 	secStop = 3.0

	# if rospy.has_param("robot_teleop/secRestart"):
	# 	secRestart = rospy.get_param("robot_teleop/secRestart")
	# else:
	# 	secRestart = 30.0

	# lin_vel = def_lvel
	# ang_vel = def_avel

	# time.sleep(1) # this sleep is just to ensure that the subscriber for the topics gets correctly initialized

	#if topics_to_record != None and topics_to_record != '':
	#	rospy.loginfo("Recording bag...")
	#	cmd = "rosbag record " + topics_to_record
	#	rosbag_proc = subprocess.Popen(cmd, stdin = subprocess.PIPE, shell = True, cwd = "/home/isrusora/html/public_html/bags/")
	#else:
	#	rospy.logwarn("Not recording a bag, since no topic was selected...")

if __name__ == '__main__':

	initialize()

	t = threading.Thread(target = compute_timeDiff)
	t.start()

	rospy.spin()
