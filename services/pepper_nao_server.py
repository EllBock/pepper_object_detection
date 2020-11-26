#!/usr/bin/python
# NOTE: must be python and not python3 since naoqi works with python 2.7

'''
This module has to provide to the other nodes an interface to interact with three
NAOqi proxies, without forcing the programmer to develop using python2 instead of python3.
So, this module implements three ROS Services Servers:
   - pepper_head_mover;
   - pepper_tts;
   - pepper_pose.
'''

import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_object_detection.srv import pepper_head_mover, pepper_head_moverResponse, pepper_tts, pepper_ttsResponse, pepper_pose, pepper_poseResponse

# This NAOqiNode wrapper class manages the three services that implements.

class NaoServer(NaoqiNode):

    # Initialization stuff
    def __init__(self):
        NaoqiNode.__init__(self,'nao_server')
        self.connectNaoQi()
        self.speech.setLanguage("English")
        pass


    def moveHead(self, req):
        '''
        This method calls "angleInterpolation" method of the NAOqi proxy named "ALMotion",
        which interpolates the req.axis (head yaw or head pitch) to req.angleLists radiants
        in req.timeLists seconds.
        '''
        debug_message = 'Respectively, the angles list and the times list recived: {}, {}'
        rospy.logdebug(debug_message.format(req.angleLists, req.timeLists))
        try:
            self.mover.angleInterpolation(req.axis, req.angleLists, req.timeLists, True)
            return pepper_head_moverResponse(True)
        except Exception, e:
            rospy.logerr("Cannot move head!")
            return pepper_head_moverResponse(False)


    def say(self,data):
        '''
        This method calls the "say" method of the NAOqi proxy named "ALTextToSpeech",
        which which makes the robot say the data.message received string.
        '''
        self.speech.say(data.message)
        debug_message = 'Pepper has said: {}'
        rospy.logdebug(debug_message.format(data.message))
        return pepper_ttsResponse(True)


    def setPose(self, req):
        '''
        This method calls the "goToPosture" method of the NAOqi proxy named "ALRobotPosture",
        which makes the robot go to the req.message posture.
        '''
        try:
            self.posture.goToPosture(req.message, 0.1)
            message = "Pepper has reached the {} posture."
            rospy.logdebug(message.format(req.message))
            return pepper_poseResponse(True)
        except Exception, e:
            message = "Pepper has not reached the {} posture."
            rospy.logerr(message.format(req.message))
            return pepper_poseResponse(False)


    def connectNaoQi(self):
        try:
            self.mover = self.get_proxy("ALMotion")
            self.m = rospy.Service('pepper_head_mover', pepper_head_mover, self.moveHead)
        except Exception, e:
            s = "Could not create proxy to ALMotion. Error was: {}"
            rospy.logerr(s.format(e))

        try:
            self.speech = self.get_proxy("ALTextToSpeech")
            self.s = rospy.Service('pepper_tts', pepper_tts, self.say)
        except Exception, e:
            s = "Could not create proxy to ALTextToSpeech. Error was: {}"
            rospy.logerr(s.format(e))

        try:
            self.posture = self.get_proxy("ALRobotPosture")
            self.p = rospy.Service('pepper_pose', pepper_pose, self.setPose)
        except Exception, e:
            s = "Could not create proxy to ALRobotPosture. Error was: {}"
            rospy.logerr(s.format(e))


if __name__=="__main__":
    pub = NaoServer()
    rospy.logdebug('NaoServer initialized')
    '''
    Need to spin, otherwise the reference to the services would be lost, and the server
    would be unable to serve requests (the handler method is a member of the class)
    '''
    rospy.spin()