# Object Detection with Pepper

## Dependencies

* ROS Melodic 
* OpenCV 4
* TensorFlow >= 2
* [rospkg](https://wiki.ros.org/rospkg) 
* A correctly set-up workspace with
  * NaoQI SDK
  * from the [ros-naoqi](https://github.com/ros-naoqi) repos
    * [pepper_robot](https://github.com/ros-naoqi/pepper_robot)
    * [naoqi_bridge](https://github.com/ros-naoqi/naoqi_bridge)
    * [naoqi_bridge_msgs](https://github.com/ros-naoqi/naoqi_bridge_msgs)
  * [vision_msgs](https://github.com/Kukanani/vision_msgs)


## Workspace setup **ADD RECOMMENDATION**
Clone this repo into your ``src`` directory, then build the workspace.


### Object Detection Model

Choose a model from the [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md). We recommend **RECOMMENDATION HERE**. 

Download and extract it somewhere. Then, change the ``detector_path`` field in ``src/pepper_object_detection/config/default.yaml`` with the model's path.


### OpenCV 4

As of now, ROS Melodic (based on Ubuntu 18.04) comes with OpenCV 3.2.0 which, unfortunately for us, has a [bug](https://github.com/opencv/opencv/issues/6969) in its image stitching module. To solve this problem, we opted to install OpenCV 4 from PyPI with the [``opencv-python``](https://pypi.org/project/opencv-python/) package.

*Any **system-wide** installation of OpenCV 4 should be fine for our intents and purposes.*

But, if you don't want to install OpenCV 4 system-wide, you can install ``opencv-python`` with ``pip3`` in a directory of your choice. For example, we usually create a ``dependencies`` directory in our workspace, and downloaad the package there.
```
$ mkdir dependencies
$ cd dependencies
$ pip3 install --target=. opencv-python
```
Then, to make this directory visible to our nodes, we change the ``dependencies_path`` field in ``src/pepper_object_detection/config/default.yaml`` to the ``dependencies`` directory. This field should be empty otherwise.


## Usage

Run everything in one line with our pre-configured launch file
```
$ roslaunch pepper_object_detection pepper_object_detection.launch pepper_ip:=<YOUR_ROBOT_IP>
```
or, if you want to test each node separately, load the configuration with
```
$ rosparam load src/pepper_object_detection/config/default.yaml
$ rosparam set /nao_server/pip "<YOUR_ROBOT_IP>"
```
then run each of the following lines in separate terminal windows

```
$ roslaunch pepper_bringup pepper_full_py.launch nao_ip:=<YOUR_ROBOT_IP>
$ rosrun pepper_object_detection pepper_nao_server.py
$ rosrun pepper_object_detection pepper_object_detection_server.py
$ rosrun pepper_object_detection master_node.py
```

# OLD PLEASE DELETE

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
rosparam load src/pepper_object_detection/config/default.yaml
rosrun pepper_object_detection pose_node.py
rosrun pepper_object_detection detector_node
rosrun pepper_object_detection pepper_nao_server.py
rosrun pepper_object_detection test_master_node.py
```

## Test image
Test image is a photo by Jonathan Cooper on Unsplash (non-copyrighted images section).

To publish the image on the /testcam/image_raw topic use this command (the publisher node will be called /testcam).
The command assumes that you are (with you shell) into the directory of this project.

```python
rosrun image_publisher image_publisher resources/test_image.jpg __name:=testcam
```