#!/usr/bin/env python3

import os
import rospy
from threading import Lock
import ros_numpy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import cv2
from pepper_object_detection.classmap import category_map as classmap
from pepper_object_detection.srv import pepper_tts
import random


def pepper_say(sentence, timeout=None):
    try:
        rospy.wait_for_service('pepper_tts', timeout)
        tts = rospy.ServiceProxy('pepper_tts', pepper_tts)
        res = tts(sentence)
        if not res:
            rospy.logerr('Unable to use TTS Server')
    except rospy.ServiceException as e:
        rospy.logerr("TTS Service call failed: %s"%e)
    except rospy.ROSException as e:
        rospy.logwarn("TTS Server request timeout")


# Init
pepper_cam_topic = rospy.get_param('pepper_cam_topic')
input_image_topic = rospy.get_param('input_image_topic')
detection_topic = rospy.get_param('detection_topic')
rospy.init_node('test_master_node')

# Test TTS
quotes = ["winter is coming", "look, a three headed monkey", 
          "to be, or not to be, that is the question", 
          "i have the high ground", "okay google", "hello world", 
          "how much wood would a woodchuck chuck if a woodchuck could chuck wood?"]
pepper_say(random.choice(quotes), 100)



# Pepper muove la testa
### TO DO ###


# Test Detection

## Image acquisition
img_msg = rospy.wait_for_message(pepper_cam_topic, Image)
rospy.loginfo('Received Image from Pepper Camera')

## Publish image to object detector
pub_img = rospy.Publisher(input_image_topic, Image, queue_size=1)
pub_img.publish(img_msg)
rospy.loginfo('Sent image Image to Object Detector')
pub_img.unregister()

## Get detections
det_msg = rospy.wait_for_message(detection_topic, Detection2DArray)
rospy.loginfo('Received Detections from Object Detector')

## Create bounding boxes on test image
image = ros_numpy.numpify(img_msg)
h,w,_ = image.shape

# TO DO: adattare per vedere più oggetti uguali
detected_objects = []
for d in det_msg.detections:
    c = d.results[0].id
    s = d.results[0].score
    b = [d.bbox.center.y,d.bbox.center.x,d.bbox.size_y, d.bbox.size_x]
    b[0]-=b[2]/2
    b[1]-=b[3]/2
    p1 = (int(b[1]*w+.5), int(b[0]*h+.5))
    p2 = (int((b[3]+b[1])*w+.5), int((b[2]+b[0])*h+.5))
    rospy.loginfo(p1, p2, c, classmap[c], s)
    col = (255,0,0) 
    cv2.rectangle(image, p1, p2, col, 3 )
    p1 = (p1[0]-10, p1[1])
    cv2.putText(image, "%s %.2f" % (classmap[c],s), p1, cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    detected_objects.append(classmap[c])

## Pepper says the list of objects
sentence = "i see"
if len(detected_objects) == 0:
    sentence = sentence + " nothing"
else:
    for o in detected_objects:
        sentence = sentence + " " + o + ","

rospy.loginfo(f"Sending \"{sentence}\" to TTS Server")
pepper_say(sentence, 100)

## Show test image
cv2.imshow('Image', image)
cv2.waitKey(0)

cv2.destroyAllWindows()
rospy.loginfo("My job here is done. Shutting down..")