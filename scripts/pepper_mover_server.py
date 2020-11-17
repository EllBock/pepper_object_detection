#!/usr/bin/python
# NOTE: must be python and not python3 since naoqi works with python 2.7
import rospy
from naoqi_driver.naoqi_node import NaoqiNode
from pepper_object_detection.srv import pepper_mover, pepper_moverResponse


class Mover(NaoqiNode):

    def __init__(self):
        NaoqiNode.__init__(self,'mover_server') # name of the ROS node, not the Aldebaran module
        self.connectNaoQi()
        pass

    def move(self, req):
        rospy.loginfo(req.angleLists)
        rospy.loginfo(req.timeLists)
        '''self.mover.setStiffnesses("Head", 1.0)
        motionProxy.angleInterpolation("HeadYaw", angleLists, timeLists, True)
        motionProxy.setStiffnesses("Head", 0.0)'''
        return pepper_moverResponse(True) # used to return a value of the type declared in the service definition

    def connectNaoQi(self):
        self.mover = self.get_proxy("ALMotion")
        self.s = rospy.Service('pepper_mover', pepper_mover, self.move)


if __name__=="__main__":
    pub = Mover()
    rospy.loginfo('Mover server initialized')
    rospy.spin()