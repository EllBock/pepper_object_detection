#!/usr/bin/python3
import rospy
from pepper_object_detection.srv import pepper_object_detection, pepper_object_detection_np, pepper_object_detection_npRequest
import cv2
from cv_bridge import CvBridge
import ros_numpy
import numpy as np
from PIL import Image


TEST_IMAGE_PATH = '/home/mivia/pepper_object_detection_ws/src/pepper_object_detection/resources/test_image.jpg'
TEST_NP = True
rospy.init_node('object_detection_server_tester')
bridge = CvBridge()
img = cv2.imread(TEST_IMAGE_PATH)
img_message = bridge.cv2_to_imgmsg(img)
try:
    rospy.wait_for_service('pepper_object_detection', 10)
    if TEST_NP:
        obj_det = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection_np)
        # img = img[:,:,::-1] # BGR -to- RGB
        req = pepper_object_detection_npRequest()
        req.img = img.tobytes()
        res = obj_det(req)
    else:
        obj_det = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection)
        res = obj_det(img_message)
    if not res:
        rospy.logerr('Unable to use Object Detection Server')
    rospy.loginfo('Detection here')
    h,w,_ = img.shape
    for d in res.detections:
        c = d.results[0].id
        s = d.results[0].score
        b = [d.bbox.center.y,d.bbox.center.x,d.bbox.size_y, d.bbox.size_x]
        b[0]-=b[2]/2
        b[1]-=b[3]/2
        p1 = (int(b[1]*w+.5), int(b[0]*h+.5))
        p2 = (int((b[3]+b[1])*w+.5), int((b[2]+b[0])*h+.5))
        col = (255,0,0) 
        cv2.rectangle(im, p1, p2, col, 3 )
        p1 = (p1[0]-10, p1[1])
        cv2.putText(im, "%s %.2f" % (classmap[c],s), p1, cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    cv2.imshow('Image', im)
    cv2.waitKey(100)

except rospy.ServiceException as e:
    rospy.logerr("Object Detection Service call failed: %s"%e)
except rospy.ROSException as e:
    rospy.logwarn("Object Detection Server request timeout")
