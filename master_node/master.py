#!/usr/bin/python3
import rospy
from threading import Lock
import ros_numpy
from sensor_msgs.msg import Image


def rcv_image(msg):
    global i
    global N
    global pub

    rospy.loginfo('Received image from Pepper')

    i += 1
    if i > N:
        i = 0
        pub.publish(msg)
        rospy.loginfo('Sent image to pipeline')


N = 1000
i = 0

pepper_cam_topic = rospy.get_param('pepper_cam_topic')
input_image_topic = rospy.get_param('input_image_topic')

rospy.init_node('master_node')

sub = rospy.Subscriber(pepper_cam_topic, Image, rcv_image, queue_size=1)
pub = rospy.Publisher(input_image_topic, Image, queue_size=1)

rospy.loginfo('Master initialized')

rospy.spin()