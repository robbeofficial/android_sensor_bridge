#!/usr/bin/env python
# Robert Walter
#   https://github.com/robbeofficial

# Run IP Webcam on your android phone
#  https://play.google.com/store/apps/details?id=com.pas.webcam&hl=de
#  use it's browser interface to configure

# TODO allow changing of parameters through ROS messages / dynamic reconf
# publish current camera settings as ros message
# TODO create class
# TODO doc: did not use dynreconf because too static
# TODO check if it's okay to use msg.String instead of srv.GetStringResponse
# TODO logging instead of print

import urllib2
import requests

#import cv2
#import numpy as np

import rospy
import std_msgs.msg
import sensor_msgs.msg

import android_sensor_bridge.srv

def setting_get(setting):
    status = requests.get(hoststr + '/status.json').json()
    return std_msgs.msg.String(status['curvals'][setting])

def setting_options(setting):
    status = requests.get(hoststr + '/status.json', params = {'show_avail': 1}).json()
    return std_msgs.msg.String(';'.join(status['avail'][setting])) # TODO catch key error

def setting_set(setting, req):
    # torch: /enabletorch /disabletorch
    # focus: /focus /nofocus
    # video_size: /settings/video_size?set=352x288

    value = req.value.data
    
    # create a list of possible entry points
    entry_points = [hoststr + '/settings/' + setting + "?set=" + value]
    if value in ['on', 'off']:
        entry_points.append(hoststr + ('enable' if value == 'on' else 'disable') + setting)
        entry_points.append(hoststr + ('' if value == 'on' else 'no') + setting)
    
    # try calling them
    for entry_point in entry_points:
        print('querying ' + entry_point)
        http = requests.get(entry_point)
        if http.status_code == 200:
            break

    resp = http.text
    return std_msgs.msg.String(resp) 
    
def add_settings_services():
    settings = requests.get(hoststr + '/status.json').json()['curvals'].keys()
    
    for setting in settings:
        if setting != " ":
            rospy.Service("~" + setting + "/get", android_sensor_bridge.srv.GetString, 
                lambda req, setting=setting : setting_get(setting)) 
            rospy.Service("~" + setting + "/options", android_sensor_bridge.srv.GetString, 
                lambda req, setting=setting : setting_options(setting))
            rospy.Service("~" + setting + "/set", android_sensor_bridge.srv.SetString, 
                lambda req, setting=setting : setting_set(setting, req)) 

if __name__ == '__main__':
    rospy.init_node('ip_webcam_bridge')
 
    # params
    host = rospy.get_param('~host', '192.168.43.1')
    port = rospy.get_param('~port', 8080)

    hoststr = 'http://{}:{}/'.format(host, port)

    # pubs
    pub_image = rospy.Publisher('~image/compressed', sensor_msgs.msg.CompressedImage, queue_size=1)

    # services
    add_settings_services()

    rospy.loginfo('Streaming ' + hoststr + '/video')
    stream = urllib2.urlopen(hoststr + '/video')

    buffer = ''
    while not rospy.is_shutdown():
        buffer += stream.read(1024)
        a = buffer.find('\xff\xd8')
        b = buffer.find('\xff\xd9')
        if a != -1 and b != -1:
            jpg = buffer[a:b+2]
            buffer = buffer[b+2:]

            # create and send CompressedImage message
            img_msg = sensor_msgs.msg.CompressedImage()
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
