#! /usr/bin/env python  

# code upload

import subprocess
import os.path
from os.path import expanduser
import os
import rospkg
import roslaunch
import rospy
from std_msgs.msg import String
import magabot.srv

home = '/home/amateus'
#home = expanduser('~')
#print "HOME", home

def handle_start_code_exec(req):
	analyse_pkg()
	return magabot.srv.CodeExecResponse('execution started')

def handle_stop_code_exec(req):
	callback()
	return magabot.srv.CodeExecResponse('execution terminated')

def callback():
#	if data.data == "stop_all":
#		exec_main_launch.terminate()
	info_str = 'Deleting pkg and shuting down'
	rospy.loginfo(info_str)
	pub_info.publish(info_str)
	subprocess.call(['rm', '-r', pkg_path])
	# TODO : FALTA MATAR OS NOS 
#	Rospy.signal_shutdown('execution terminated')

def analyse_pkg():
	global pkg_path

	# msg = String()
	# msg.data = 'stop_all'
	
	# mv from upload folder to ros_ws

        
#	subprocess.call(['ls -l', '/home/amateus/html/public_html/uploads'])
	
	# ATTENTION ABS PATH DEFINED
	rospy.loginfo(home)
	subprocess.call(['ls', home+'/html/public_html/uploads'])
        
        str1 = home+"/html/public_html/uploads/package"
        str2 = home + "/catkin_ws/src/"
        str3 = 'mv'
        rospy.loginfo(str1)
        rospy.loginfo(str2)
	subprocess.call([str3, str1, str2])


	# get the path of the ros package
	rospack = rospkg.RosPack()
	pkg_path = rospack.get_path('package')

	# list all the contents of the package folder
	ls_list = subprocess.check_output(['ls', pkg_path]).splitlines()

	# all pre-requisites of the packege
	req_list = ['CMakeLists.txt', 'launch', 'nodes', 'package.xml']

	# verifying specs of the package
	info_str = 'Verifying the package structure'
	rospy.loginfo(info_str)
	pub_info.publish(info_str)

	for x in req_list:
		if not x in ls_list:
			error_str = "ERROR: " + x + ' missing'
			rospy.logerr(error_str)
			pub_err.publish(error_str)
			subprocess.call(['rm', '-r', pkg_path])
			callback()

	launch_idx = ls_list.index('launch')

	if 'nodes' in ls_list:
		nodes_idx = ls_list.index('nodes')
	elif 'scripts' in ls_list:
		nodes_idx = ls_list.index('scripts')

	nodes_path = pkg_path + '/' + ls_list[nodes_idx]

	if not os.path.isfile(pkg_path+'/'+ls_list[launch_idx]+'/main.launch'):
		error_str = "ERROR: main.launch missing"
		rospy.logerr(error_str)
		pub_err.publish(error_str)
		subprocess.call(['rm', '-r', pkg_path])
		callback()
	
	del_temp_files = subprocess.Popen('cd '+ nodes_path + '&& rm *~', shell = True, stdout = subprocess.PIPE)
	del_temp_files.wait()

	for file in os.listdir(nodes_path):
		if not file.endswith('.py'):
			error_str = "ERROR: nodes must be written in python"
			rospy.logerr(error_str)
			pub_err.publish(error_str)
			subprocess.call(['rm', '-r', pkg_path])
			callback()       

	# all nodes executable
	subprocess.call(['chmod', '-R', '+x', nodes_path])

	info_str = 'Launching main.launch'
	rospy.loginfo(info_str)
	pub_info.publish(info_str)

	exec_launch = subprocess.Popen('roslaunch package main.launch', shell=True)
	#os.system('roslaunch package main.launch')


if __name__ == "__main__":

	global exec_main_launch, pub_info, pub_err

	rospy.init_node('code_execution')
	# rospy.Subscriber("action", String, callback)

	rospy.Service('start_code_exec', magabot.srv.CodeExec, handle_start_code_exec)
	rospy.Service('stop_code_exec', magabot.srv.CodeExec, handle_stop_code_exec)

        pub_info = rospy.Publisher('/magabot/loginfo', String) 
	pub_err = rospy.Publisher('/magabot/logerr', String)

	# analyse_pkg()
	
	rospy.spin()
