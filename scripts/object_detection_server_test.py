#!/usr/bin/python3
import rospy
from pepper_object_detection.srv import pepper_object_detection
import cv2
from cv_bridge import CvBridge


TEST_IMAGE_PATH = '/home/mivia/pepper_object_detection_ws/src/pepper_object_detection/resources/test_image.jpg'
bridge = CvBridge()
img = cv2.imread(TEST_IMAGE_PATH)
img_message = bridge.cv2_to_imgmsg(img)
try:
    rospy.wait_for_service('pepper_object_detection', 10)
    obj_det = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection)
    res = obj_det(img_message)
    if not res:
        rospy.logerr('Unable to use Object Detection Server')
    print(res.result)
except rospy.ServiceException as e:
    rospy.logerr("Object Detection Service call failed: %s"%e)
except rospy.ROSException as e:
    rospy.logwarn("Object Detection Server request timeout")
