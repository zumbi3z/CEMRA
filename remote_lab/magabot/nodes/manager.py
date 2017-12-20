#! /usr/bin/env python

import os
import rospy
from std_msgs.msg import *
from magabot.msg import Topics



def manager(msg):
	
	rospy.loginfo(msg)

	if str(msg.data) == "start_teleop":
		rospy.loginfo("--- Launching teleoperation ---")
		os.system("roslaunch magabot_joy teleop.launch")
	elif str(msg.data) == "start_all":
		rospy.loginfo("--- Launching code execution and teleoperation---")
		os.system("roslaunch magabot code_execution.launch")
		#raw_input()
		#rospy.loginfo("--- Launching teleoperation ---")
		#os.system("roslaunch magabot_joy teleop.launch")
	elif str(msg.data) == "start_bag":
		rospy.loginfo("--- Launching record bag ---")
		os.system("roslaunch magabot record_bag.launch")
	else:
		pass


if __name__ == "__main__":
	global pub

	rospy.init_node("manager_node")
	rospy.Subscriber("action", String, manager)
	
	pub = rospy.Publisher("action", String)

	rospy.spin()
