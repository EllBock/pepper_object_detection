#!/usr/bin/python
# NOTE: must be python and not python3 since naoqi works with python 2.7
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_object_detection.srv import pepper_tts, pepper_ttsResponse


class AnimatedSay(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self,'tts_server') # name of the ROS node, not the Aldebaran module
        self.connectNaoQi()
        self.speech.setLanguage("English")
        pass

    def say(self,data):
        rospy.loginfo("START: %s" % data.message)
        self.speech.say(data.message)
        rospy.loginfo("END: %s" % data.message) # data.message contains the exact string received
        # rospy.loginfo(data.message) # TEST
        return pepper_ttsResponse(True) # used to return a value of the type declared in the service definition

    def connectNaoQi(self):
        self.speech=self.get_proxy("ALTextToSpeech")
        self.s = rospy.Service('pepper_tts', pepper_tts, self.say)


if __name__=="__main__":
    pub = AnimatedSay()
    rospy.loginfo('TTS server initialized')
    rospy.spin()