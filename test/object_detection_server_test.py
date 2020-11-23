#!/usr/bin/python3

"""
    This is a module for testing the node implementing the Object Detection Service Server.
"""

import rospy
from pepper_object_detection.srv import pepper_object_detection, pepper_object_detection_np, pepper_object_detection_npRequest
import cv2
from cv_bridge import CvBridge
import ros_numpy
import numpy as np
from PIL import Image
from pepper_object_detection.classmap import category_map as classmap


TEST_IMAGE_PATH = '/home/mivia/pepper_object_detection_ws/src/pepper_object_detection/resources/test_image.jpg'
# Choose wheter you want to test the server which receives numpy arrays into the request (legacy) or the one which receives
# sensor_msgs/Image instances.
TEST_NP = False
rospy.init_node('object_detection_server_tester')
bridge = CvBridge()
# Read the image into a numpy array
img = cv2.imread(TEST_IMAGE_PATH)
# Create a copy of the image in a format compatible with sensor_msgs/Image
img_message = bridge.cv2_to_imgmsg(img)
try:
    rospy.wait_for_service('pepper_object_detection', 10)
    if TEST_NP:
        obj_det = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection_np)
        req = pepper_object_detection_npRequest()
        # Since ROS doesn't allow to send matrices as messages, the image matrix is serialized into a bytes array
        req.img = img.tobytes()
        # Send the request to the server
        res = obj_det(req)
    else:
        obj_det = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection)
        # Send the request to the server
        res = obj_det(img_message)
    if not res:
        rospy.logerr('Unable to use Object Detection Server')
        exit()
    rospy.loginfo('Detection received')
    image_height,image_width,_ = img.shape
    for detection in res.detections.detections:
        class_id = detection.results[0].id
        detection_score = detection.results[0].score
        box = [detection.bbox.center.y,detection.bbox.center.x,detection.bbox.size_y, detection.bbox.size_x]
        box[0]-=box[2]/2
        box[1]-=box[3]/2
        # Find bounding box's vertices (upper left, lower right) and draw the box on the image
        p1 = (int(box[1]*image_width+.5), int(box[0]*image_height+.5))
        p2 = (int((box[3]+box[1])*image_width+.5), int((box[2]+box[0])*image_height+.5))
        col = (255,0,0) 
        cv2.rectangle(img, p1, p2, col, 3 )
        p1 = (p1[0]-10, p1[1])
        # Write the label on the box
        cv2.putText(img, "%s detection_score %.2f" % (classmap[class_id],detection_score), p1, cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Image', 600, 600)
    cv2.imshow('Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

except rospy.ServiceException as e:
    rospy.logerr("Object Detection Service call failed: %detection_score"%e)
except rospy.ROSException as e:
    rospy.logwarn("Object Detection Server request timeout")
