#!/usr/bin/env python3

import sys
import rospy
import ros_numpy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from pepper_object_detection.classmap import category_map as classmap
from pepper_object_detection.srv import pepper_tts, pepper_object_detection, pepper_head_mover, pepper_pose
import random
from cv_bridge import CvBridge

TRIAL_NUMBER_FAILURE = 10
DEBUG = True

# Initializing ROS node
pepper_cam_topic = rospy.get_param('pepper_cam_topic')
cv2_dir = rospy.get_param('dependencies_path')
rospy.init_node('master_node')

if cv2_dir != "":
    sys.path.insert(0, cv2_dir)

import cv2

# Stitcher does not work properly with OpenCV<=3.2.0
cv_version = cv2.__version__.split(".")
if int(cv_version[0]) < 3 or (int(cv_version[0]) == 3 and int(cv_version[1]) <= 2):
    rospy.logfatal("OpenCV {cv2.__version__} is not supported. OpenCV > 3.2.0 required. Exiting..")
    exit()
elif int(cv_version[0]) < 4:
    rospy.logwarn("This code has not been tested with OpenCV < 4. Please, beware of unexpected behaviour.")



# A simpler interface to the image stiching algorithm from OpenCV
def stitch(images):
    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    return stitcher.stitch(images)


# Function to generate a sentence with objects
# which counts objects and separates them with a separator
def objects_sentence(objects: dict, separator=", "):
    if len(objects) == 0:
        return "nothing"

    sentence = ""
    keys = list(objects.keys())

    # We don't want the order of objects to be arbitrary or predefined
    random.shuffle(keys)
    
    # For each class of objects, we print the number and the class
    for k in keys:
        n = objects[k]
        if n > 1:
            # Naive approach to plural
            o = k + "s"
        else:
            o = k
        sentence = sentence + f"{n} {o}" + separator

    # Deleting the last separator from the sentence
    sentence = sentence[:-len(separator)] 

    return sentence



# We wait for each service to be online
try:
    rospy.loginfo("Waiting for TTS service...")
    rospy.wait_for_service('pepper_tts', 20)
    tts_proxy = rospy.ServiceProxy('pepper_tts', pepper_tts)
    rospy.loginfo("TTS service OK")

    rospy.loginfo("Waiting for Head Movement service...")
    rospy.wait_for_service('pepper_head_mover', 20)
    head_mover_proxy = rospy.ServiceProxy('pepper_head_mover', pepper_head_mover)
    rospy.loginfo("Head Movement service OK")

    rospy.loginfo("Waiting for Pose service...")
    rospy.wait_for_service('pepper_pose', 20)
    pose_proxy = rospy.ServiceProxy('pepper_pose', pepper_pose)
    rospy.loginfo("Pose service OK")

    rospy.loginfo("Testing Camera input...")
    rospy.wait_for_message(pepper_cam_topic, Image, 20)
    rospy.loginfo("Camera topic OK")

    rospy.loginfo("Waiting for Object Detection service...")
    rospy.wait_for_service('pepper_object_detection', 200)
    detector_proxy = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection)
    rospy.loginfo("Object Detector service OK")

except rospy.ROSException as e:
    rospy.logerr("Request timeout. Exiting..")
    exit()



# The robot goes to a neutral position
try:
    res = pose_proxy('StandInit')
    if not res:
        i = 0
        while(not res):
            rospy.logwarn(f"Could not reach required pose! Trying again ({i+1}) ..")
            res = pose_proxy('StandInit')
            i += 1
            if i > TRIAL_NUMBER_FAILURE:
                rospy.logerr("Could not reach required pose! Exiting..")
                exit()
except rospy.ServiceException as e:
    rospy.logerr("Go To Posture call failed: %s"%e)
    exit()

rospy.loginfo("Setup completed.")


# We then acquire images from a series of positions
img_msgs = []

# We define a series of absolute positions for the robot's head.
# Format: (Axis, Angle (rad), Time (s), Take photo?)

# 10 immagini, 0.4 di scarto
positions = [("HeadYaw", 0.8, 1.6, False),
             ("HeadPitch", -0.2, 1.2, True),
             ("HeadYaw", 0.4, 0.8, True),
             ("HeadYaw", 0.0, 0.8, True),
             ("HeadYaw", -0.4, 0.8, True),
             ("HeadYaw", -0.8, 0.8, True),
             ("HeadPitch", 0.2, 1.2, True),
             ("HeadYaw", -0.4, 0.8, True),
             ("HeadYaw", 0.0, 0.8, True),
             ("HeadYaw", 0.4, 0.8, True),
             ("HeadYaw", 0.8, 0.8, True),
             ("HeadYaw", 0.0, 0.8, False),
             ("HeadPitch", 0.0, 1.2, False)]

# We follow the trajectory and catch images from the 
# camera's topic when the flag is True
rospy.loginfo("Scanning the area...")
for p in positions:
    angleLists = []
    angleLists.append(p[1])
    timeLists = []
    timeLists.append(p[2])
    try:
        res = head_mover_proxy(p[0], angleLists, timeLists)
        if not res:
            i = 0
            while(not res):
                rospy.logwarn(f"Could not move head! Trying again ({i+1}) ..")
                res = head_mover_proxy(p[0], angleLists, timeLists)
                i += 1
                if i > TRIAL_NUMBER_FAILURE:
                    rospy.logerr("Could not move head! Exiting..")
                    exit()
    except rospy.ServiceException as e:
        rospy.logerr("Move Head call failed: %s"%e)
        exit()
    
    if p[3]:
        try:
            img_msgs.append(rospy.wait_for_message(pepper_cam_topic, Image, 10))
        except rospy.ROSException as e:
            rospy.logerr("Camera input timeout. Exiting...")
            exit() 


# We execute the image stitching
images = []
for i in range(len(img_msgs)):
    images.append(ros_numpy.numpify(img_msgs[i]))

res, panorama = stitch(images)

# We exit if there's an error with the stitching
if panorama is None or res != 0:
    rospy.logerr(f"Image Stitching failed: status {res}. Exiting..")
    exit()


# For debug purposes, we save the panorama image.
if DEBUG:
    cv2.imshow("Panorama View", panorama)
    cv2.waitKey(100)
    cv2.imwrite('panorama.jpg', panorama)

# We generate an Image message
rospy.loginfo("Detecting objects...")
bridge = CvBridge()
pnr_msg = bridge.cv2_to_imgmsg(panorama)

# that we send to the detector
det = detector_proxy(pnr_msg).detections


# We divide the area in left, center and right areas
h,w,_ = panorama.shape
left_center_bound = int(w/3)
center_right_bound = int(2*w/3)

# For each area, we save each object as a key
# with value the number of objects of the same class found in the same area
objects = {'sx':{}, 'center':{}, 'dx':{}}


for d in det.detections:
    # We read detection results
    c = d.results[0].id
    s = d.results[0].score
    bbcenter_x = d.bbox.center.x * w
    b = [d.bbox.center.y,d.bbox.center.x,d.bbox.size_y, d.bbox.size_x]
    b[0]-=b[2]/2
    b[1]-=b[3]/2
    p1 = (int(b[1]*w+.5), int(b[0]*h+.5))
    p2 = (int((b[3]+b[1])*w+.5), int((b[2]+b[0])*h+.5))
    
    # We check the area where the center of a bounding box is
    if bbcenter_x < left_center_bound:
        area = 'sx'
    elif bbcenter_x < center_right_bound:
        area = 'center'
    else:
        area = 'dx'
    
    # We insert the object in the right dict
    ob_class = classmap[c]
    if ob_class in objects[area]:
        objects[area][ob_class] += 1
    else:
        objects[area][ob_class] = 1

    rospy.logdebug(f"Found {ob_class}: {p1}, {p2} ({area}); score {s}")

    # For debug purposes, we add the bbs to the panorama image
    if DEBUG:
        col = (255,0,0) 
        cv2.rectangle(panorama, p1, p2, col, 2)
        p1 = (p1[0]-10, p1[1])
        cv2.putText(panorama, "%s %.2f" % (ob_class,s), p1, cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)


# We generate a sentence
sentence = "on the left, i see " + objects_sentence(objects['sx']) + ";"
sentence = sentence + " in front of me, i see " + objects_sentence(objects['center']) + ";"
sentence = sentence + " on the right, i see " + objects_sentence(objects['dx']) + ";"

# We send the sentence to the tts
rospy.loginfo(f"TTS: {sentence}")
try:
    res = tts_proxy(sentence)
    i = 0
    while(not res):
        rospy.logwarn(f"Could not make Pepper speak! Trying again ({i+1}) .. ")
        res = tts_proxy(sentence)
        i += 1
        if i > TRIAL_NUMBER_FAILURE:
            rospy.logerr("Could not make Pepper speak! Exiting..")
            exit()
except rospy.ServiceException as e:
    rospy.logerr("TTS Service call failed: %s"%e)
    exit()

# For debug purposes we can show the image on screen
# and save it on the filesystem.
if DEBUG:
    panorama = cv2.line(panorama, (left_center_bound, 0), (left_center_bound, h), (0, 255, 0), 2)
    panorama = cv2.line(panorama, (center_right_bound, 0), (center_right_bound, h), (0, 255, 0), 2)
    cv2.imshow("Panorama Detection", panorama)
    cv2.waitKey(0)
    cv2.imwrite("panorama_od.jpg", panorama)
    cv2.destroyAllWindows()


rospy.loginfo("Task completed. Closing...")