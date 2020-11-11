# Object Detection with Pepper

## Workspace setup

1. Clone this package and into your ros workspace ``src`` folder.
2. Download and extract an object detection model from the [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md).
    E.g. EfficientDet D1:
    ```bash
    wget http://download.tensorflow.org/models/object_detection/tf2/20200711/efficientdet_d1_coco17_tpu-32.tar.gz
    tar -xvzf efficientdet_d1_coco17_tpu32.tar.gz
    ```
3. Change ``config/default.yaml`` accordingly.
