#!/usr/bin/python

from pepper_object_detection.srv import pepper_mover
import rospy

mover = rospy.ServiceProxy('pepper_mover', pepper_mover)
print('[test] pepper_mover_initialized')
res = mover([1.0], [2.0])
print('[test] pepper_tts_server_result: {}'.format(res.result)) # res.result contains the exact returned value            