#!/usr/bin/env python

import rospy
from naoqi import ALProxy


rospy.init_node('pose_node')


try:
    postureProxy = ALProxy("ALRobotPosture", robotIP, 9559)
except Exception, e:
    rospy.logerror(f"Could not create proxy to ALRobotPosture. Error was: {e}")
    exit() 

rospy.loginfo("Pose Node initialized.")
postureProxy.goToPosture("StandInit", 1.0)
rospy.loginfo("Stand Pose reached.")

try:
    rospy.spin()
except KeyboardInterrupt:
    postureProxy.goToPosture("Crouch", 1.0)
    rospy.loginfo("Going to sleep..")
    exit()


