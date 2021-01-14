#!/usr/bin/python3

"""
    This module implements a ROS Service Server for object detection.
    The Service is described in srv/pepper_object_detection.srv . It allows to provide an image to this node and receive,
    for each object detected into the image, the corresponding bounding box, class ID and confidence score.
"""

import rospy
from pepper_object_detection.srv import pepper_object_detection, pepper_object_detectionResponse 
from pepper_object_detection.detector import Detector
import ros_numpy
from pepper_object_detection.classmap import category_map as classmap
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import logging

class PepperObjectDetectorService():
    """
        This class takes care of managing the lifecycle of the ROS Object Detection Service, by starting and stopping
        the service and serving requests submitted to it.
    """

    def __init__(self, detector_path):
        """
        Args:
            detector_path (string): path to the folder containing the object recognition model. Must be in a format compatible
            with tensorflow.saved_model.load (https://www.tensorflow.org/api_docs/python/tf/saved_model/load). Note that the 
            path must point to the folder, not the model, since the loading function automatically searches for supported file types.
        """
        self._detection_model = Detector(detector_path)
        self._service = rospy.Service('pepper_object_detection', pepper_object_detection, self.detect_objects)
        pass

    def detect_objects(self, data):
        """
        This function detects and classifies the objects in the image provided through a Service Request by running on her the provided
        detection model. Returns a vision_msgs/Detection2DArray, for which each detection is populated only with the bbox and results fields. Moreover,
        for what it concerns the results field, each result is populated only with the id and score fields.
        All the other fields are not significant for this application, so they have been ignored.

        Args:
            data (sensor_msgs/Image): image to perform object detection on.

        Returns:
            pepper_object_detectionResponse: response encapsulating data regarding detected objects, structured as in service definition.
        """

        # Convert image from sensor_msgs/Image to numpy array
        img_np = ros_numpy.numpify(data.img) 
        rospy.loginfo('Object detection server computing predictions...')
        detections = self._detection_model(img_np)
        rospy.loginfo('Predictions computed!')
        message = Detection2DArray()
        # Insert all the predictions into the message and return them
        for class_id,score,box in zip(detections['detection_classes'], detections['detection_scores'], detections['detection_boxes']):
            detection = Detection2D()
            detection.bbox.size_x = box[3]-box[1]
            detection.bbox.size_y = box[2]-box[0]
            detection.bbox.center.x = box[1]+detection.bbox.size_x/2
            detection.bbox.center.y = box[0]+detection.bbox.size_y/2
            detected_object = ObjectHypothesisWithPose()
            detected_object.score = score
            detected_object.id = class_id
            detection.results.append(detected_object)
            message.detections.append(detection)
        # Create a response object
        response = pepper_object_detectionResponse() 
        response.detections = message
        return response
    
    def stop(self, reason = 'User request'):
        """
        Stops the Object Detection Service server.

        Args:
            reason (str, optional): The reason why the service is being stopped. Defaults to 'User request'.
        """
        rospy.loginfo('Shutting down object detection server...')
        self._service.shutdown(reason)
        return


if __name__ == '__main__':
    rospy.init_node('object_detection_server')
    # Filter non-fatal warnings from TensorFlow (otherwise we would get tons of non-critical warnings regarding gradients
    # when loading the model)
    logging.getLogger('tensorflow').setLevel(logging.FATAL)
    detector_service = PepperObjectDetectorService(rospy.get_param('detector_path')+'/saved_model') # the path to the model is loaded from the parameter server
    # Register callback to shutdown service when the server node is stopped 
    rospy.on_shutdown(detector_service.stop)
    rospy.loginfo('Object detection server initialized')
    # Need to spin, otherwise the reference to the PepperObjectDetectorService would be lost, and the server
    # would be unable to serve requests (the handler method is a member of the class)
    rospy.spin()