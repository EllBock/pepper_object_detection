#!/usr/bin/python
# NOTE: must be python and not python3 since naoqi works with python 2.7
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_object_detection.srv import pepper_mover, pepper_moverResponse, pepper_tts, pepper_ttsResponse, pepper_pose, pepper_poseResponse


class NaoServer(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self,'nao_server') # name of the ROS node, not the Aldebaran module
        self.connectNaoQi()
        #self.speech.setLanguage("English")
        pass


    def move(self, req):
        rospy.loginfo(req.angleLists)
        rospy.loginfo(req.timeLists)
        try:
            self.mover.setStiffnesses("Head", 1.0)
            self.mover.angleInterpolation("HeadYaw", angleLists, timeLists, True)
            self.mover.setStiffnesses("Head", 0.0)
            return pepper_moverResponse(True)
        except Exception, e:
            rospy.logerr("Cannot move head!")
            return pepper_moverResponse(False)


    def say(self,data):
        rospy.loginfo("START: %s" % data.message)
        #self.speech.say(data.message)
        rospy.loginfo("END: %s" % data.message) # data.message contains the exact string received
        # rospy.loginfo(data.message) # TEST
        return pepper_ttsResponse(True) # used to return a value of the type declared in the service definition


    def get_pose(self, req):
        try:
            self.posture.goToPosture(req, 1.0)
            message = "Pepper has reached the {} posture."
            rospy.loginfo(message.format(req))
            return pepper_poseResponse(True)
        except Exception, e:
            message = "Pepper has not reached the {} posture."
            rospy.loginfo(message.format(req))
            return pepper_poseResponse(False)


    def connectNaoQi(self):
        try:
            self.mover = self.get_proxy("ALMotion")
        except Exception, e:
            s = "Could not create proxy to ALMotion. Error was: {}"
            rospy.logerr(s.format(e))

        try:
            self.speech = self.get_proxy("ALTextToSpeech")
        except Exception, e:
            s = "Could not create proxy to ALTextToSpeech. Error was: {}"
            rospy.logerr(s.format(e))

        try:
            self.posture = self.get_proxy("ALRobotPosture")
        except Exception, e:
            s = "Could not create proxy to ALRobotPosture. Error was: {}"
            rospy.logerr(s.format(e))

        self.m = rospy.Service('pepper_mover', pepper_mover, self.move)
        self.s = rospy.Service('pepper_tts', pepper_tts, self.say)
        self.p = rospy.Service('pepper_pose', pepper_pose, self.get_pose)


if __name__=="__main__":
    pub = NaoServer()
    rospy.loginfo('NaoServer initialized')
    rospy.spin()