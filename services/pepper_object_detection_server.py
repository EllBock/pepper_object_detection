#!/usr/bin/python3

import rospy
from pepper_object_detection.srv import pepper_object_detection, pepper_object_detectionResponse 
from pepper_object_detection.detector import Detector
import ros_numpy
from pepper_object_detection.classmap import category_map as classmap
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import logging


class PepperObjectDetectorService():

    def __init__(self, detector_path):
        self._detection_model = Detector(detector_path)
        self._s = rospy.Service('pepper_object_detection', pepper_object_detection, self.detect_objects)
        pass

    def detect_objects(self, data):
        # Convert image from sensor_msgs/Image to numpy array
        img_np = ros_numpy.numpify(data.img) 

        rospy.loginfo('Object detection server computing predictions...')
        detections = self._detection_model(img_np)
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
        response.detections = message
        return response
    
    def stop(self, reason = 'User request'):
        rospy.loginfo('Shutting down object detection server...')
        self._s.shutdown(reason)
        return


if __name__ == '__main__':
    rospy.init_node('object_detection_server')
    logging.getLogger('tensorflow').setLevel(logging.FATAL)
    detector_service = PepperObjectDetectorService(rospy.get_param('detector_path')+'/saved_model')
    # Register callback to shut down service when the server node is stopped 
    rospy.on_shutdown(detector_service.stop)
    rospy.loginfo('Object detection server initialized')
    rospy.spin()