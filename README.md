# Object Detection with Pepper

## Workspace setup

1. Clone this package and into your ros workspace ``src`` folder.
2. Download and extract an object detection model from the [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md).
    E.g. EfficientDet D1:
    ```bash
    wget http://download.tensorflow.org/models/object_detection/tf2/20200711/efficientdet_d1_coco17_tpu-32.tar.gz
    tar -xvzf efficientdet_d1_coco17_tpu32.tar.gz
    ```
3. Change the detector model path in ``config/default.yaml`` accordingly.
4. ```bash
   echo 'export PYTHONPATH=${PYTHONPATH}:/home/mivia/cogrob_git/pynaoqi-python2.7-2.5.7.1-linux64/lib/python2.7/site-packages' >> devel/setup.bash
   echo 'export DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:$/home/mivia/cogrob_git/pynaoqi-python2.7-2.5.7.1-linux64/lib' >> devel/setup.bash
   ```

## Testing commands
```python
roslaunch pepper_bringup pepper_full_py.launch nao_ip:=10.0.1.230
rosparam load $(find pepper_object_detection)/config/default.yaml
rosrun pepper_object_detection pose_node.py
rosrun pepper_object_detection detector_node
rosrun pepper_object_detection pepper_tts_server.py
rosrun pepper_object_detection test_master_node.py
```

## Test image
Test image is a photo by Jonathan Cooper on Unsplash (non-copyrighted images section).

