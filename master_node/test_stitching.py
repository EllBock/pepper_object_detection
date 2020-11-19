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
from master_utils import pepper_say, objects_sentence, detect_objects, move_head
from stitcher import stitch
from cv_bridge import CvBridge

# Init
pepper_cam_topic = rospy.get_param('pepper_cam_topic')
input_image_topic = rospy.get_param('input_image_topic')
detection_topic = rospy.get_param('detection_topic')
rospy.init_node('master_node')

# Image acquisition
img_msgs = []
positions = [1.6, 0.8, 0.0, -0.8, -1.6]
last_position = 0.0

for p in positions:
    angleLists = []
    angleLists.append(p)
    timeLists = []
    timeLists.append(abs(p - last_position))
    move_head(angleLists, timeLists, 100)
    last_position = p
    img_msgs.append(rospy.wait_for_message(pepper_cam_topic, Image))

move_head([0.0], [abs(0.0 - last_position)])


# Stitching
images = []
for i in range(len(img_msgs)):
    images.append(ros_numpy.numpify(img_msgs[i]))

panorama = stitch(images)

# Object Detection
bridge = CvBridge()
pnr_msg = bridge.cv2_to_imgmsg(panorama)
det = detect_objects(pnr_msg, 100)

# Show detections
h,w,_ = panorama.shape
pnr_cpy = panorama.deepcopy()
for d in det.detections:
    c = d.results[0].id
    s = d.results[0].score
    b = [d.bbox.center.y,d.bbox.center.x,d.bbox.size_y, d.bbox.size_x]
    b[0]-=b[2]/2
    b[1]-=b[3]/2
    p1 = (int(b[1]*w+.5), int(b[0]*h+.5))
    p2 = (int((b[3]+b[1])*w+.5), int((b[2]+b[0])*h+.5))
    rospy.loginfo(f"Found {classmap[c]}: {p1}, {p2}; score {s}")
    col = (255,0,0) 
    cv2.rectangle(pnr_cpy, p1, p2, col, 3 )
    p1 = (p1[0]-10, p1[1])
    cv2.putText(pnr_cpy, "%s %.2f" % (classmap[c],s), p1, cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)


cv2.imshow("Panorama", pnr_cpy)
cv2.waitKey(0)

cv2.destroyAllWindows()
rospy.loginfo("My job here is done. Shutting down..")