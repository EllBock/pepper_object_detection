#!/usr/bin/python3
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_object_detection.srv import pepper_tts


class AnimatedSay(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self,'animated_speech')
        self.connectNaoQi()
        pass

    def say(self,data):
        rospy.loginfo("START: %s" % data.message)
        self.speech.say(data.message)
        rospy.loginfo("END: %s" % data.message)
        return SayResponse(True)

    def connectNaoQi(self):
        self.speech=self.get_proxy("ALAnimatedSpeech")
        self.s = rospy.Service('pepper_tts', pepper_tts, self.say)


if __name__=="__main__":
    pub = AnimatedSay()
    rospy.loginfo('TTS server initialized')
    rospy.spin()