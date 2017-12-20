#! /usr/bin/env python

import rospy
import os
import os.path
from os.path import expanduser
import time
import datetime
import subprocess
import stat

home='/home/amateus'
#home = expanduser('~')


if __name__ == '__main__':
    rospy.init_node('remove_bags')

    bags_folder = home + "/html/public_html/bags/"
    bag_list = []
    while not rospy.is_shutdown():
        list_cmd = subprocess.Popen("ls " + bags_folder, shell = True, stdout = subprocess.PIPE)
        list_output = list_cmd.stdout.read()
        retcode = list_cmd.wait()
        assert retcode == 0, rospy.logwarn("List command returned %d" % retcode)
        for s in list_output.split("\n"):
            if s != '':
                t =  time.time() - os.stat(bags_folder + s)[stat.ST_CTIME]
                if t >= 86400: # 24h in seconds
                    rm_bag = subprocess.Popen('rm -rf ' + bags_folder + s, shell = True)
                    rm_bag.wait()
                    rospy.logwarn('Bag ' + s + ' removed!')
        

    
    
