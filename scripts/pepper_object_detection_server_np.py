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
        detections = self._detection_model(rcvd_img)
        rospy.loginfo('Predictions computed!')
        message = Detection2DArray()
        for clabel,score,box in zip(detections['detection_classes'], detections['detection_scores'], detections['detection_boxes']):
            d = Detection2D()
            d.bbox.size_x = box[3]-box[1]
            d.bbox.size_y = box[2]-box[0]
            d.bbox.center.x = box[1]+d.bbox.size_x/2
            d.bbox.center.y = box[0]+d.bbox.size_y/2
            o = ObjectHypothesisWithPose()
            o.score = score
            o.id = clabel
            d.results.append(o)
            message.detections.append(d)
        # Create a response object
        response = pepper_object_detectionResponse() 
        response.detections = detections
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