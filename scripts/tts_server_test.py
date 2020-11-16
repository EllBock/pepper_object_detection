#!/usr/bin/python

from pepper_object_detection.srv import pepper_tts
import rospy

tts = rospy.ServiceProxy('pepper_tts', pepper_tts)
print('[test] pepper_tts_initialized')
res = tts('prova')
print('[test] pepper_tts_server_result: {}'.format(res.result)) # res.result contains the exact returned value            