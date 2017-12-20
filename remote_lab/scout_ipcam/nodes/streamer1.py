
import sys
import urllib
import urlparse
import cStringIO as StringIO

from PIL import Image

import re

import fix_bad_unicode as fbu

PKG_NAME = 'scout_ipcam'
import roslib; roslib.load_manifest(PKG_NAME)
import rospy

from sensor_msgs.msg import *
import numpy as np


DEFAULT_URL = "http://127.0.0.1/video.mjpg"


def main(url = DEFAULT_URL):
    cimg = CompressedImage()
    #cimg = Image()
    cimg.format = 'jpeg'
    
    if "cgi" in url:
    	cimg.format = 'jpg'

    # ROS initialization
    rospy.init_node(PKG_NAME, anonymous = True)
	
    rospy.loginfo("Argument url: %s" % (url))

    name_topic = "/camera_driver_" + urlparse.urlparse(url).netloc.split(".")[3] + "_" + re.search(r'\d+', urlparse.urlparse(url).path).group() + "/stream/compressed"

    if int(re.search(r'\d+', urlparse.urlparse(url).path).group()) == 1:
	if "cgi" in url:
		url = "http://" + urlparse.urlparse(url).netloc + "/cgi-bin/video.jpg?size=3"
	else:
		url = "http://" + urlparse.urlparse(url).netloc + "/video.mjpg"
	rospy.loginfo(url)

    pub_camera_info = rospy.Publisher("/camera1/camera_info", CameraInfo)
    camera_info = CameraInfo()
    camera_info.header.seq = 0
    camera_info.header.stamp = rospy.Time.now()
    camera_info.header.frame_id = "camera1"
    camera_info.height = 216
    camera_info.width = 384
    #camera_info.height = 480
    #camera_info.width = 612
    camera_info.distortion_model = "plumb_bob"
    camera_info.D = np.float64([-0.32339, 0.09856, -0.00117, -0.00108, 0])
    #camera_info.D = np.float64([-0.263486880572991, 0.06070720777843115, -0.005405289503419933, -0.0037001675771910674, 0.0])
    camera_info.K = np.float64([191.77583, 0, 193.40582, 0, 191.63449, 108.52168, 0, 0, 1])
    #camera_info.K = np.float64([1058.106795218165, 0.0, 946.9857891497771, 0.0, 1056.1268520857298, 518.406617016872, 0.0, 0.0, 1.0])
    camera_info.R = np.float64([1, 0, 0, 0, 1, 0, 0, 0, 1])
    camera_info.P = np.float64([191.77583, 0, 193.40582, 0, 0, 191.63449, 108.52168, 0, 0, 0, 1.0000, 0])
    #camera_info.P = np.float64([783.9783935546875, 0.0, 914.7922684114455, 0.0, 0.0, 973.94091796875, 507.971482642286, 0.0, 0.0, 0.0, 1.0, 0.0])
    rospy.sleep(0.5)
    #print camera_info
    rospy.loginfo("Publishing the camera info topic...")
    pub_camera_info.publish(camera_info)
    
    
    pub = rospy.Publisher(name_topic, CompressedImage)
    
    # Connecting to IP camera and start streaming
    rospy.loginfo("Opening source %s for streaming" % (url))
    fh  = urllib.urlopen(url)
	
    #print type(fh)
    #print fh.headers
    #print fh.readlines()
    #print dir(fh)

    size = None
    seq = 1
    while not rospy.is_shutdown():
        while True:
            line = fh.readline()		

        #    if line=='':
        #        return
        #    line = line.strip()
          #  print "LINE(%s)='%s'"%(len(line), line)
          #  rospy.sleep(3)
            if line.startswith("Content-Length"):
                fs = line.split()
                try:
                    size = int(fs[1])
                    line = fh.readline()
                    break
                except StandardError:
		    continue
            elif line == '' and size is not None:
		    break
  #      print "HEADER END"

	# Read image
        data = fh.read(size)
        #data = fh.readlines()
        
        # Publish jpeg image
        cimg.data = data
        pub.publish(cimg)

        camera_info.header.seq = seq
        seq += 1
        camera_info.header.stamp = rospy.Time.now()
        pub_camera_info.publish(camera_info)

        rospy.sleep(1.0/30)
        size = None
        





if __name__=='__main__':
    main()

# EOF
