#!/usr/bin/env python3

import os
import rospy
import ros_numpy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import cv2
from pepper_object_detection.classmap import category_map as classmap
from pepper_object_detection.srv import pepper_tts, pepper_object_detection, pepper_head_mover
import random
from cv_bridge import CvBridge

TRIAL_NUMBER_FAILURE = 10
DEBUG = True


def stitch(images):
    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    return stitcher.stitch(images)[1]


def objects_sentence(objects: dict, separator=", "):
    if len(objects) == 0:
        return "nothing"

    sentence = ""
    keys = list(objects.keys())
    random.shuffle(keys)
    for k in keys:
        n = objects[k]
        if n > 1:
            o = k + "s"
        else:
            o = k
        sentence = sentence + f"{n} {o}" + separator

    return sentence[:-len(separator)] 



# Init
pepper_cam_topic = rospy.get_param('pepper_cam_topic')
rospy.init_node('master_node')

if cv2.__version__ != "4.4.0":
    rospy.logfatal("OpenCV {cv2.__version__} is not supported. Exiting..")
    exit()

rospy.loginfo("Waiting for services...")
try:
    rospy.wait_for_service('pepper_tts', 100)
    tts_proxy = rospy.ServiceProxy('pepper_tts', pepper_tts)
    rospy.loginfo("TTS OK")
    rospy.wait_for_service('pepper_head_mover', 100)
    head_mover_proxy = rospy.ServiceProxy('pepper_head_mover', pepper_head_mover)
    rospy.loginfo("Head Movement OK!")
    rospy.wait_for_service('pepper_object_detection', 500)
    detector_proxy = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection)
    rospy.loginfo("Object Detector OK!")
except rospy.ROSException as e:
    rospy.logerr("Service connection timeout. Exiting..")
    exit()

rospy.loginfo("Services ready.")



# Image acquisition
img_msgs = []
# (Axis, Angle (rad), Time (s), Take photo?)
positions = [("HeadYaw", 0.8, 0.8, False),
             ("HeadPitch", 0.1, 0.1, True),
             ("HeadYaw", 0.4, 0.4, True),
             ("HeadYaw", 0.0, 0.4, True),
             ("HeadYaw", -0.4, 0.4, True),
             ("HeadYaw", -0.8, 0.4, True),
             ("HeadPitch", -0.2, 0.3, True),
             ("HeadYaw", -0.4, 0.4, True),
             ("HeadYaw", 0.0, 0.4, True),
             ("HeadYaw", 0.4, 0.4, True),
             ("HeadYaw", 0.8, 0.4, True),
             ("HeadYaw", 0.0, 0.8, False),
             ("HeadPitch", 0.0, 0.2, False)]

for p in positions:
    angleLists = []
    angleLists.append(p[1])
    timeLists = []
    timeLists.append(p[2])
    res = head_mover_proxy(p[0], angleLists, timeLists)
    try:
        if not res:
            i = 0
            while(not res):
                rospy.logwarn(f"Could not move head! Trying again ({i+1}) ..")
                res = head_mover_proxy(p[0], angleLists, timeLists)
                i += 1
                if i > TRIAL_NUMBER_FAILURE:
                    rospy.logerr("Could not move head! Exiting..")
                    exit()
    except rospy.ServiceException as e:
        rospy.logerr("Move Head call failed: %s"%e)
        exit()
    
    if p[3]:
        img_msgs.append(rospy.wait_for_message(pepper_cam_topic, Image))


# Image Stitching
images = []
for i in range(len(img_msgs)):
    images.append(ros_numpy.numpify(img_msgs[i]))

panorama = stitch(images)

if DEBUG:
    cv2.imshow("Panorama", panorama)
    cv2.waitKey(0)
    cv2.imwrite('panorama.jpg', panorama)
    cv2.destroyAllWindows()

# Object Detection
bridge = CvBridge()
pnr_msg = bridge.cv2_to_imgmsg(panorama)
det = detector_proxy(pnr_msg).detections


# Sentence Generation
h,w,_ = panorama.shape
left_center_bound = int(w/3)
center_right_bound = int(2*w/3)

objects = {'sx':{}, 'center':{}, 'dx':{}}


for d in det.detections:
    c = d.results[0].id
    s = d.results[0].score
    bbcenter_x = d.bbox.center.x
    b = [d.bbox.center.y,d.bbox.center.x,d.bbox.size_y, d.bbox.size_x]
    b[0]-=b[2]/2
    b[1]-=b[3]/2
    p1 = (int(b[1]*w+.5), int(b[0]*h+.5))
    p2 = (int((b[3]+b[1])*w+.5), int((b[2]+b[0])*h+.5))
    
    if bbcenter_x < left_center_bound:
        area = 'sx'
    elif bbcenter_x < center_right_bound:
        area = 'center'
    else:
        area = 'dx'
    
    ob_class = classmap[c]
    if ob_class in objects[area]:
        objects[area][ob_class] += 1
    else:
        objects[area][ob_class] = 1

    rospy.logdebug(f"Found {ob_class}: {p1}, {p2} ({area}); score {s}")
    if DEBUG:
        col = (255,0,0) 
        cv2.rectangle(panorama, p1, p2, col, 2)
        p1 = (p1[0]-10, p1[1])
        cv2.putText(panorama, "%s %.2f" % (ob_class,s), p1, cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)

sentence = "on the left, i see " + objects_sentence(objects['sx']) + "."
sentence = sentence + " in front of me, i see " + objects_sentence(objects['center']) + "."
sentence = sentence + " on the right, i see " + objects_sentence(objects['dx']) + "."

if DEBUG:
    panorama = cv2.line(panorama, (left_center_bound, 0), (left_center_bound, h), (0, 255, 0), 2)
    panorama = cv2.line(panorama, (center_right_bound, 0), (center_right_bound, h), (0, 255, 0), 2)
    cv2.imshow("Panorama", panorama)
    cv2.waitKey(0)
    cv2.imwrite("panorama_od.jpg", panorama)
    cv2.destroyAllWindows()

try:
    res = tts_proxy(sentence)
    i = 0
    while(not res):
        rospy.logwarn(f"Could not make Pepper speak! Trying again ({i+1}) .. ")
        res = tts_proxy(sentence)
        i += 1
        if i > TRIAL_NUMBER_FAILURE:
            rospy.logerr("Could not make Pepper speak! Exiting..")
            exit()
except rospy.ServiceException as e:
    rospy.logerr("TTS Service call failed: %s"%e)
    exit()



rospy.loginfo("Task completed. Closing..")