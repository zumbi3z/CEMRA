
import sys
import urllib
import urlparse
import cStringIO as StringIO
from PIL import Image as IM
import re

import fix_bad_unicode as fbu

PKG_NAME = 'scout_ipcam'
import roslib; roslib.load_manifest(PKG_NAME)
import rospy

from sensor_msgs.msg import *

DEFAULT_URL = "http://127.0.0.1/video.mjpg"

def main(url = DEFAULT_URL):
    cimg = CompressedImage()
    cimg.format = 'jpeg'
    
    if "cgi" in url:
    	cimg.format = 'jpg'

    # ROS initialization
    rospy.init_node(PKG_NAME, anonymous = True)
	
    #rospy.loginfo("Argument url: %s" % (url))

    name_topic = "/camera_driver_" + urlparse.urlparse(url).netloc.split(".")[3] + "_" + re.search(r'\d+', urlparse.urlparse(url).path).group() + "/stream/compressed"

    if int(re.search(r'\d+', urlparse.urlparse(url).path).group()) == 1:
	if "cgi" in url:
		url = "http://" + urlparse.urlparse(url).netloc + "/cgi-bin/video.jpg?size=3"
	else:
		url = "http://" + urlparse.urlparse(url).netloc + "/video.mjpg"
	rospy.loginfo(url)
    
    pub = rospy.Publisher(name_topic, CompressedImage)
    
    # Connecting to IP camera and start streaming
    rospy.loginfo("Opening source %s for streaming" % (url))
    #fh  = urllib.urlopen(url)
	
    #print type(fh)
    #print fh.headers
    #print fh.readlines()
    #print dir(fh)
    
    
    pub_camera_info = rospy.Publisher("/camera2/camera_info", CameraInfo)
    camera_info = CameraInfo()
    camera_info.header.seq = 0
    camera_info.header.stamp = rospy.Time.now()
    camera_info.header.frame_id = "camera1"
    camera_info.height = 216
    camera_info.width = 384
    #camera_info.height = 480
    #camera_info.width = 612
    camera_info.distortion_model = "plumb_bob"
    rospy.sleep(0.5)
    pub_camera_info.publish(camera_info)
    
    
    
    seq = 1
    size = None
    while not rospy.is_shutdown():
	#rospy.loginfo("Opening source %s for streaming" % (url))
	fh  = urllib.urlopen(url)

        #while True:
            #line = fh.readline()		

        ##    if line=='':
        ##        return
        ##    line = line.strip()
          ##  print "LINE(%s)='%s'"%(len(line), line)
          ##  rospy.sleep(3)
            #if line.startswith("Content-Length"):
                #fs = line.split()
                #try:
                    #size = int(fs[1])
                    #line = fh.readline()
                    #break
                #except StandardError:
		    #continue
            #elif line == '' and size is not None:
		    #break
  ##      print "HEADER END"

	# Read image
	#print "SIZE", size
        #data = fh.read(size)
        data = fh.readlines()
        
        #print data
        #raw_input()
        
        #for entry in data:
	  #print "DATA", entry.decode('ascii')
	  #rospy.sleep(1.0)
        
        #img = Image.new("L", (704, 576), "white")
        #print dir(PIL)
        #img = IM.open('/home/isrusora/Desktop/video.jpg')
	#img.putdata(data)
        
        #print dir(img)
        
        #img.show()
        
        #raw_input()
        
        # Publish jpeg image
        d = "".join(data)
        #print type(d)
        #raw_input()
        cimg.data = d
        
        #print type(cimg)
        
        pub.publish(cimg)
        
        camera_info.header.seq = seq
        seq += 1
        camera_info.header.stamp = rospy.Time.now()
        pub_camera_info.publish(camera_info)
        
        rospy.sleep(1.0/30)
        size = None
	 
        #rospy.loginfo('Image published')
        
        #fh.close()
        #rospy.sleep(1.0)





if __name__=='__main__':
    main()

# EOF
