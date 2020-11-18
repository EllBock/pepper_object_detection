#!/usr/bin/python

from pepper_object_detection.srv import pepper_tts, pepper_mover, pepper_pose
import rospy

# Test TTS

tts = rospy.ServiceProxy('pepper_tts', pepper_tts)
print('[test] pepper_tts_initialized')
res = tts('prova')
print('[test] pepper_tts_server_result: {}'.format(res.result)) # res.result contains the exact returned value

# Test Mover

mover = rospy.ServiceProxy('pepper_mover', pepper_mover)
print('[test] pepper_mover_initialized')
res = mover([1.0], [2.0])
print('[test] pepper_tts_server_result: {}'.format(res.result))

# Test Pose

tts = rospy.ServiceProxy('pepper_pose', pepper_pose)
print('[test] pepper_pose_initialized')
res = tts('StandInit')
print('[test] pepper_pose_server_result: {}'.format(res.result))