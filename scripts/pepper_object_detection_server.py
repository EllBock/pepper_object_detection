#!/usr/bin/python3

import rospy
from pepper_object_detection.srv import pepper_object_detection, pepper_object_detectionResponse 
from pepper_object_detection.detector import Detector
import ros_numpy
from pepper_object_detection.classmap import category_map as classmap


class PepperObjectDetectorService():

    def __init__(self, detector_path):
        self._detection_model = Detector(detector_path)
        self._s = rospy.Service('pepper_object_detection', pepper_object_detection, self.detect_objects)
        pass

    def detect_objects(self, data):
        # Convert image from sensor_msgs/Image to numpy array
        img_np = ros_numpy.numpify(data.img) 

        rospy.loginfo('Object detection server computing predictions...')
        predictions = self._detection_model(img_np)
        rospy.loginfo('Predictions computed!')
        # Create a response object
        response = pepper_object_detectionResponse() 
        # Populate it with detected classes
        for detected_class in predictions['detection_classes']:
            response.result.append(classmap[detected_class])
        return response
    
    def stop(self, reason = 'User request'):
        rospy.loginfo('Shutting down object detection server...')
        self._s.shutdown(reason)
        return


if __name__ == '__main__':
    rospy.init_node('object_detection_server')
    detector_service = PepperObjectDetectorService(rospy.get_param('detector_path')+'/saved_model')
    # Register callback to shut down service when the server node is stopped 
    rospy.on_shutdown(detector_service.stop)
    rospy.loginfo('Object detection server initialized')
    rospy.spin()