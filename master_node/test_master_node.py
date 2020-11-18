#!/usr/bin/env python3

import os
import rospy
from threading import Lock
import ros_numpy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import cv2
from pepper_object_detection.classmap import category_map as classmap
from pepper_object_detection.srv import pepper_tts, pepper_object_detection, pepper_head_mover
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
        rospy.logwarn("TTS Server request timeout.")


def detect_objects(imgmsg, timeout=None):
    try:
        rospy.wait_for_service('pepper_object_detection', timeout)
        det = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection)
        return det(imgmsg)
    except rospy.ServiceException as e:
        rospy.logerr("Object Detection call failed: %s"%e)
    except rospy.ROSException as e:
        rospy.logwarn("Object Detection Server request timeout.")


def move_head(angleLists, timeLists, timeout=None)
    try:
        rospy.wait_for_service('pepper_mover', timeout)
        mover = rospy.ServiceProxy('pepper_mover', pepper_head_mover)
        res = mover(angleLists, timeLists)
        if not res:
            rospy.logerr('Unable complete head movement.')
    except rospy.ServiceException as e:
        rospy.logerr("Move Head call failed: %s"%e)
    except rospy.ROSException as e:
        rospy.logwarn("Head Mover Server request timeout.")


def objects_sentence(objects: dict, separator=", "):
    sentence = ""
    keys = random.shuffle(list(objects))
    for k in keys:
        n = objects[k]
        if n > 1:
            o = k + "s"
        else
            o = k
        sentence = sentence + f"{n} {o}" + separator

    return sentence[:-len(separator)] 



# Init
pepper_cam_topic = rospy.get_param('pepper_cam_topic')
input_image_topic = rospy.get_param('input_image_topic')
detection_topic = rospy.get_param('detection_topic')
rospy.init_node('test_master_node')

# Test TTS
quotes = ["winter is coming", "look, a three headed monkey", 
          "to be, or not to be, that is the question", 
          "i am the senate", "okay google, phone home", "hello world", 
          "how much wood would a woodchuck chuck if a woodchuck could chuck wood?",
          "never gonna give you up, never gonna let you down",
          "the cake is a lie"]
pepper_say(random.choice(quotes), 100)



# Image acquisition
img_msgs = []
move_head(1.5, 1.5)
img_msgs.append(rospy.wait_for_message(pepper_cam_topic, Image))
move_head(0.0, 1.5)
img_msgs.append(rospy.wait_for_message(pepper_cam_topic, Image))
move_head(-1.5, 0.0)
img_msgs.append(rospy.wait_for_message(pepper_cam_topic, Image))


# Object Detection
## Getting detections on all images
det_msgs = []
for i in range(len(img_msgs)):
    det_msgs.append(detect_objects(img_msgs[i], timeout=100))

## Showing the test images
detections = []

for i in range(len(img_msg)):
    image = ros_numpy.numpify(img_msgs[i]))
    detected_objects = {}
    h,w,_ = image.shape

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
        if classmap[c] in detected_objects:
            detected_objects[classmap[c]] += 1
        else:
            detected_objects[classmap[c]] = 1

    detections.append(detected_objects)


## Pepper says the list of objects
sentence = "on the left i see" + objects_sentence(detections[0]) + "."
sentence = sentence + " in the middle i see" + objects_sentence(detections[1]) + "."
sentence = sentence + " on the right i see" + objects_sentence(detections[2]) + "."

rospy.loginfo(f"Sending \"{sentence}\" to TTS Server")
pepper_say(sentence, 100)

## Show test image
cv2.imshow('Image', image)
cv2.waitKey(0)

cv2.destroyAllWindows()
rospy.loginfo("My job here is done. Shutting down..")