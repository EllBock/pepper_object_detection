import rospy
import random
from pepper_object_detection.srv import pepper_tts, pepper_object_detection, pepper_head_mover

def pepper_say(sentence, timeout=None):
    try:
        rospy.wait_for_service('pepper_tts', timeout)
        tts = rospy.ServiceProxy('pepper_tts', pepper_tts)
        res = tts(sentence)
        if not res:
            rospy.logerr('Unable to use TTS Server')
    except rospy.ServiceException as e:
        rospy.logerr("TTS Service call failed: %s"%e)
    except rospy.ROSException as e:
        rospy.logwarn("TTS Service request timeout.")


def detect_objects(imgmsg, timeout=None):
    try:
        rospy.wait_for_service('pepper_object_detection', timeout)
        det = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection)
        return det(imgmsg).detections
    except rospy.ServiceException as e:
        rospy.logerr("Object Detection call failed: %s"%e)
    except rospy.ROSException as e:
        rospy.logwarn("Object Detection Service request timeout.")


def move_head(axis, angleLists, timeLists, timeout=None):
    try:
        rospy.wait_for_service('pepper_head_mover', timeout)
        mover = rospy.ServiceProxy('pepper_head_mover', pepper_head_mover)
        res = mover(axis, angleLists, timeLists)
        if not res:
            rospy.logerr('Unable to complete head movement.')
    except rospy.ServiceException as e:
        rospy.logerr("Move Head call failed: %s"%e)
    except rospy.ROSException as e:
        rospy.logwarn("Head Mover Service request timeout.")


def objects_sentence(objects: dict, separator=", "):
    if len(objects) == 0:
        return "nothing"

    sentence = ""
    keys = list(objects.keys())
    random.shuffle(keys)
    for k in keys:
        n = objects[k]
        if n > 1:
            o = k + "s"
        else:
            o = k
        sentence = sentence + f"{n} {o}" + separator

    return sentence[:-len(separator)] 
