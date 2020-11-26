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


## Workspace setup
Clone this repo into your ``src`` directory, then build the workspace.


### Object Detection Model

Choose a model from the [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md). We recommend **Faster R-CNN ResNet50 V1 800x1333**. 

Download and extract it somewhere. Then, change the ``detector_path`` field in ``src/pepper_object_detection/config/default.yaml`` with the model's path.


### OpenCV 4

As of now, ROS Melodic (based on Ubuntu 18.04) comes with OpenCV 3.2.0 which, unfortunately for us, has a [bug](https://github.com/opencv/opencv/issues/6969) in its image stitching module. To solve this problem, we opted to install OpenCV 4 from PyPI with the [``opencv-python``](https://pypi.org/project/opencv-python/) package.

*Any **system-wide** installation of OpenCV 4 should be fine for our intents and purposes.*

But, if you don't want to install OpenCV 4 system-wide, you can install ``opencv-python`` with ``pip3`` in a directory of your choice. For example, we usually create a ``dependencies`` directory in our workspace, and download the package there.
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