import cv2
import tensorflow as tf
assert(int(tf.__version__.split('.')[0]) >= 2)
import numpy as np
import time

"""
    This class implements an object detector. It loads a TensorFlow model form the provided path and uses it to
    perform inference on the input the Detector is called on.
    Note that this class is explicitly designed to work (only) with TensorFlow object detection models.
"""
class Detector():
    def __init__(self,model_path):
        print('Trying to load model from ', model_path)
        self.detect_fn = tf.saved_model.load(model_path)

    def __call__(self, img, threshold=0.5):
        # The execution time measurement used in this method is unsafe (wrt to SIGINT or reference time change for example) 
        # and it's only an indicative value
        start_time = time.time()
        # BGR -to- RGB
        img = img[:,:,::-1]  
        # Convert to the tensor type required by TensorFlow
        input_tensor = tf.convert_to_tensor(img)
        input_tensor = input_tensor[tf.newaxis, ...]
        detections = self.detect_fn(input_tensor)
        num_above_thresh = np.sum( detections['detection_scores'] > threshold )
        print ("%d objects found" % num_above_thresh)
        # Put the detections dictionary in a format easier to work with
        detections.pop('num_detections')
        detections = {key: value[0, :num_above_thresh].numpy() for key, value in detections.items()}
        detections['detection_classes'] = detections['detection_classes'].astype(np.int64)
        end_time = time.time()
        print('Computing predictions required %.2f seconds' % (end_time-start_time))
        return detections