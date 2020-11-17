#!/usr/bin/env python

import rospy
from naoqi import ALProxy


rospy.init_node('pose_node')

try:
    postureProxy = ALProxy("ALRobotPosture", rospy.get_param("/tts_server/pip"), 9559)
except Exception, e:
    s = "Could not create proxy to ALRobotPosture. Error was: {}"
    rospy.logerr(s.format(e))
    exit()


def shutdown():
    postureProxy.goToPosture("Crouch", 1.0)
    rospy.loginfo("Going to sleep..")


rospy.on_shutdown(shutdown)

rospy.loginfo("Pose Node initialized.")
postureProxy.goToPosture("StandInit", 1.0)
rospy.loginfo("Stand Pose reached.")

rospy.spin()

    

