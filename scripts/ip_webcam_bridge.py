#!/usr/bin/env python
# Robert Walter
#   https://github.com/robbeofficial

import urllib2

#import cv2
#import numpy as np

import rospy
#from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

# Run IP Webcam on your android phone
#  https://play.google.com/store/apps/details?id=com.pas.webcam&hl=de
#  use it's browser interface to configure

# TODO allow changing of parameters through ROS messages / dynamic reconf
# publish current camera settings as ros message
# TODO create class

if __name__ == '__main__':

    #print cv2.__version__

    rospy.init_node('ip_webcam_bridge')

    # pubs
    pub_image = rospy.Publisher('~image/compressed', CompressedImage, queue_size=1)

    # params
    host = rospy.get_param('~host', '192.168.178.46')
    port = rospy.get_param('~port', 8080)

    # data stream
    hoststr = 'http://{}:{}/video'.format(host, port)
    rospy.loginfo('Streaming ' + hoststr)
    stream = urllib2.urlopen(hoststr)

    buffer = ''
    while not rospy.is_shutdown():
        buffer += stream.read(1024)
        a = buffer.find('\xff\xd8')
        b = buffer.find('\xff\xd9')
        if a != -1 and b != -1:
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]

            # create and send CompressedImage message
            img_msg = CompressedImage()
            img_msg.data = jpg
            img_msg.format = 'jpeg'
            pub_image.publish(img_msg)
            rospy.logdebug('published image')

            # decode and show image
            #img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.CV_LOAD_IMAGE_COLOR) # openCV < 3
            #img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR) # openCV >= 3
            #cv2.imshow(hoststr, img)
            #if cv2.waitKey(1) == 27:
            #    exit(0)
