#!/usr/bin/python3

import rospy
from pepper_object_detection.srv import pepper_object_detection_np, pepper_object_detection_npResponse
from pepper_object_detection.detector import Detector
import ros_numpy
from pepper_object_detection.classmap import category_map as classmap
import numpy as np
import cv2
from PIL import Image


class PepperObjectDetectorService():

    def __init__(self, detector_path):
        self._detection_model = Detector(detector_path)
        self._s = rospy.Service('pepper_object_detection', pepper_object_detection_np, self.detect_objects)
        pass

    def detect_objects(self, data):
        rospy.loginfo('Object detection server (np) computing predictions...')
        img_array = np.frombuffer(data.img, dtype=np.uint8)
        rcvd_img = img_array.reshape((4753, 3395, 3))
        # DEBUG
        #window = cv2.namedWindow('rcvd_img', cv2.WINDOW_GUI_NORMAL)
        #cv2.resizeWindow('rcvd_img', 600,600)
        #cv2.imshow('rcvd_img', rcvd_img)
        #cv2.waitKey(0)
        #cv2.destroyAllWindows()
        # END DEBUG
        predictions = self._detection_model(rcvd_img)
        rospy.loginfo('Predictions computed!')
        # Create a response object
        response = pepper_object_detection_npResponse() 
        # Populate it with detected classes
        for detected_class in predictions['detection_classes']:
            response.result.append(classmap[detected_class])
        return response
    
    def stop(self, reason = 'User request'):
        rospy.loginfo('Shutting down object detection server (np) ...')
        self._s.shutdown(reason)
        return


if __name__ == '__main__':
    rospy.init_node('object_detection_server_np')
    detector_service = PepperObjectDetectorService(rospy.get_param('detector_path')+'/saved_model')
    # Register callback to shut down service when the server node is stopped 
    rospy.on_shutdown(detector_service.stop)
    rospy.loginfo('Object detection server (np) initialized')
    rospy.spin()