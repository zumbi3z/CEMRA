#! /usr/bin/env python

import rospy
#from sensor_msgs.msg import *
#from geometry_msgs.msg import *
from std_msgs.msg import String
#import threading
#import math
import subprocess
import os
from os.path import expanduser
from magabot_joy.msg import Topics
import time
import datetime
import rospkg
import string
import random
import magabot.srv

home='/home/amateus'
#home = expanduser('~')

def handle_start_rosbag(req):
    save_topic_names(req)
    return magabot.srv.StartRosbagResponse("rosbag recording")

def save_topic_names(req):
    global bag_name, package_path #, topic_names

    # topic_names = ''
    # for topic in msg.topics:
    #     topic_names += topic + ' '

    # if topic_names != '':
    date = datetime.datetime.now()
    bag_name = 'bag_' + str(date.day) + str(date.month) + str(date.year) + '_' + str(date.hour) + str(date.minute) + str(date.second)
        
    rospy.loginfo("Recording bag...")
    info_str = "Recording bag..."
    pub_info.publish(info_str)
        # cmd = "rosbag record -O " + bag_name + ' '  + topic_names
        
    topic_list = choose_topics()
    pub_info.publish(topic_list)

    ### MISSING PICK RIGHT TOPICS FOR THE CAMERAS    
    if req.cam1 == 'T' and req.cam2 == 'T':
        cmd = "rosbag record -O " + bag_name + " --split --size=2030 " + topic_list + ' /camera1/image_raw /camera2/image_raw'
    elif req.cam1 == 'F' and req.cam2 == 'F':
        cmd = "rosbag record -O " + bag_name + " --split --size=2030" + topic_list
    elif req.cam1 == 'T' and req.cam2 == 'F':
        cmd = "rosbag record -O " + bag_name + " --split --size=2030 " + topic_list + ' /camera1/image_raw'
    elif req.cam1 == 'F' and req.cam2 == 'T':
        cmd = "rosbag record -O " + bag_name + " --split --size=2030 " + topic_list + ' /camera2/image_raw'
    else:
        rospy.logerr('Camera topics incorrectly defined')
        err_str = 'Camera topics incorrectly defined'
        pub_err.publish(err_str)
        
        return
    print cmd
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('magabot')
    bags_folder_path = package_path + '/bags/'
    create_bag_folder_cmd = subprocess.Popen('mkdir ' + bags_folder_path, shell = True, stdout = subprocess.PIPE)
    create_bag_folder_cmd.wait()
    rosbag_proc = subprocess.Popen(cmd, stdin = subprocess.PIPE, shell = True, cwd = bags_folder_path)
    
    # else:
    #     rospy.logwarn("Not recording a bag, since no topic was selected...")

def handle_stop_rosbag(req):
    [link, user, password] = stop_recording()
    return magabot.srv.StopRosbagResponse(link, user, password)

def stop_recording():

#    if str(msg.data) == 'stop_bag' or str(msg.data) == 'stop_teleop' or str(msg.data) == 'stop_all':

    list_cmd = subprocess.Popen("rosnode list", shell = True, stdout = subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, rospy.logwarn("List command returned %d" % retcode)
    for s in list_output.split("\n"):
        print s
        if (s.startswith("/record")):
            os.system("rosnode kill " + s)
            break
            
    rospy.sleep(0.5)
    
    [link, user, password] = create_new_folder()

    return link, user, password
    # rospy.signal_shutdown("Stopping bag recording...")

def choose_topics():
    list_cmd = subprocess.Popen("rostopic list", shell = True, stdout = subprocess\
                                .PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, rospy.logwarn("List command returned %d" % retcode)
    topic_list = ''
    for s in list_output.split("\n"):
        if ('camera' in s):
            pass
        else:
            topic_list = topic_list + ' ' + s
    return topic_list


def user_generator(size = 6, chars = string.ascii_uppercase + string.ascii_lowercase):
    return ''.join(random.choice(chars) for _ in range(size))


def password_generator(size = 8, chars = string.ascii_uppercase + string.ascii_lowercase  + string.digits):
    return ''.join(random.choice(chars) for _ in range(size))


def create_new_folder(new_directory_path = home + '/html/public_html/bags/'):
    create_folder_cmd = subprocess.Popen('mkdir ' + new_directory_path + bag_name, shell = True, stdout = subprocess.PIPE)
    
    new_user = user_generator()
    new_password = password_generator()

    info_str = "NEW USER -> " + new_user
    print info_str
    pub_info.publish(info_str)
    info_str = "NEW PASSWORD -> " + new_password
    pub_info.publish(info_str)

    create_user_cmd = subprocess.Popen("htpasswd -b /home/amateus/html/passwd/passwords " + new_user + " " + new_password, shell = True, stdout = subprocess.PIPE)

    new_htaccess_file_path = new_directory_path + bag_name + '/' + '.htaccess'
    protect_folder_cmd = subprocess.Popen('echo AuthType Basic >> ' +  new_htaccess_file_path, shell = True, stdout = subprocess.PIPE)
    protect_folder_cmd = subprocess.Popen('echo AuthUserFile /home/amateus/html/passwd/passwords >> ' +  new_htaccess_file_path, shell = True, stdout = subprocess.PIPE)
    protect_folder_cmd = subprocess.Popen('echo Require user ' + new_user + ' >> ' +  new_htaccess_file_path, shell = True, stdout = subprocess.PIPE)

#    move_bag_cmd = subprocess.Popen('mv ' + package_path + '/bags/' + bag_name + '.bag'  + ' ' + new_directory_path + bag_name + '/', shell = True, stdout = subprocess.PIPE)

    move_bag_cmd = subprocess.Popen('mv ' + package_path + '/bags/' + bag_name + '*'  + ' ' + new_directory_path + bag_name + '/', shell = True, stdout = subprocess.PIPE)
    
    link = "sscpc0.ist.utl.pt:28080/bags/" + bag_name

    info_str = "LINK -> " + link
    pub_info.publish(info_str)

    return link, new_user, new_password


if __name__ == '__main__':
    global pub_info, pub_err

    rospy.init_node('bag_record')

    rospy.Service('start_rosbag', magabot.srv.StartRosbag, handle_start_rosbag)
    rospy.Service('stop_rosbag', magabot.srv.StopRosbag, handle_stop_rosbag)

    pub_info = rospy.Publisher('/magabot/loginfo', String)
    pub_err = rospy.Publisher('/magabot/logerr', String)

    rospy.spin()
