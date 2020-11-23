#!/usr/bin/env python3

import rospy
import ros_numpy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
import cv2
from pepper_object_detection.classmap import category_map as classmap
from pepper_object_detection.srv import pepper_tts, pepper_object_detection, pepper_head_mover, pepper_pose
import random
from cv_bridge import CvBridge

TRIAL_NUMBER_FAILURE = 10
DEBUG = True


# Interfaccia semplificata allo Stitcher di CV2
def stitch(images):
    stitcher = cv2.Stitcher_create(cv2.Stitcher_PANORAMA)
    return stitcher.stitch(images)


# Funzione per generare una lista di oggetti
# dividendoli con un separatore e tenendo conto del loro numero
def objects_sentence(objects: dict, separator=", "):
    if len(objects) == 0:
        return "nothing"

    sentence = ""
    keys = list(objects.keys())

    # Evitiamo che l'ordine degli oggetti sia arbitrario e costante
    random.shuffle(keys)
    
    # Per ogni classe, il robot dice il numero di oggetti e la classe
    for k in keys:
        n = objects[k]
        if n > 1:
            # Un plurale naive
            o = k + "s"
        else:
            o = k
        sentence = sentence + f"{n} {o}" + separator

    # Eliminiamo l'ultimo separatore dalla frase
    sentence = sentence[:-len(separator)] 

    return sentence




# Inizializzazione del nodo ROS
pepper_cam_topic = rospy.get_param('pepper_cam_topic')
rospy.init_node('master_node')


# Lo stitcher non funziona con OpenCV <3.2.0
cv_version = cv2.__version__.split(".")
if int(cv_version[0]) < 3 or (int(cv_version[0]) == 3 and int(cv_version[1]) <= 2):
    rospy.logfatal("OpenCV {cv2.__version__} is not supported. OpenCV > 3.2.0 required. Exiting..")
    exit()
elif int(cv_version[0]) < 4:
    rospy.logwarn("This code has not been tested with OpenCV < 4. Please, beware of unexpected behaviour.")


# Aspettiamo che tutti i servizi necessari siano online
rospy.loginfo("Waiting for services...")
try:
    rospy.wait_for_service('pepper_tts', 100)
    tts_proxy = rospy.ServiceProxy('pepper_tts', pepper_tts)
    rospy.loginfo("TTS OK")
    rospy.wait_for_service('pepper_head_mover', 100)
    head_mover_proxy = rospy.ServiceProxy('pepper_head_mover', pepper_head_mover)
    rospy.loginfo("Head Movement OK!")
    rospy.wait_for_service('pepper_object_detection', 500)
    detector_proxy = rospy.ServiceProxy('pepper_object_detection', pepper_object_detection)
    rospy.loginfo("Object Detector OK!")
    rospy.wait_for_service('pepper_pose', 100)
    pose_proxy = rospy.ServiceProxy('pepper_pose', pepper_pose)
    rospy.loginfo("Pepper pose manager OK!")
except rospy.ROSException as e:
    rospy.logerr("Service connection timeout. Exiting..")
    exit()

rospy.loginfo("Services ready.")


# Portiamo il robot in una posizione neutra
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


# Acquisizione delle immagini
img_msgs = []

# Definiamo una serie di posizioni assolute che la testa del robot
# deve assumere.
# Format: (Axis, Angle (rad), Time (s), Take photo?)
'''
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
'''
# 8 immagini, 0.5 di scarto
positions = [("HeadPitch", -0.2, 1.2, False),
             ("HeadYaw", 0.25, 0.8, True),
             ("HeadYaw", 0.75, 0.8, True),
             ("HeadPitch", 0.2, 1.2, True),
             ("HeadYaw", 0.25, 0.8, True),
             ("HeadYaw", -0.25, 0.8, True),
             ("HeadYaw", -0.75, 0.8, True),
             ("HeadPitch", -0.2, 0.8, True),
             ("HeadYaw", -0.25, 0.8, True),
             ("HeadYaw", 0.0, 0.8, False),
             ("HeadPitch", 0.0, 1.2, False)]


# Eseguiamo i movimenti richiesti, salvando l'immagine pubblicata sul topic
# della camera del robot quando il flag è True.
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
        img_msgs.append(rospy.wait_for_message(pepper_cam_topic, Image))


# Eseguiamo lo stitching delle immagini
images = []
for i in range(len(img_msgs)):
    images.append(ros_numpy.numpify(img_msgs[i]))

res, panorama = stitch(images)

# In caso di errore, usciamo perché è probabilmente un problema 
# delle posizioni scelte per la testa
if panorama is None or res != 0:
    rospy.logerr(f"Image Stitching failed: status {res}. Exiting..")
    exit()


# Nel caso mostriamo e salviamo una copia del risultato
if DEBUG:
    cv2.imshow("Panorama", panorama)
    cv2.waitKey(0)
    cv2.imwrite('panorama.jpg', panorama)
    cv2.destroyAllWindows()


# Generiamo un messaggio Image
bridge = CvBridge()
pnr_msg = bridge.cv2_to_imgmsg(panorama)

# che viene inviato al servizio di detection
det = detector_proxy(pnr_msg).detections


# Dividiamo la scena nelle tre zone: sinistra, centro e destra
h,w,_ = panorama.shape
left_center_bound = int(w/3)
center_right_bound = int(2*w/3)

# Per ogni zona salveremo gli oggetti come chiavi in un dizionario,
# associati al numero dello stesso tipo di oggetto in una determinata zona
objects = {'sx':{}, 'center':{}, 'dx':{}}


for d in det.detections:
    # Manipoliamo i risultati della detection
    c = d.results[0].id
    s = d.results[0].score
    bbcenter_x = d.bbox.center.x * w
    b = [d.bbox.center.y,d.bbox.center.x,d.bbox.size_y, d.bbox.size_x]
    b[0]-=b[2]/2
    b[1]-=b[3]/2
    p1 = (int(b[1]*w+.5), int(b[0]*h+.5))
    p2 = (int((b[3]+b[1])*w+.5), int((b[2]+b[0])*h+.5))
    
    # Controlliamo dove ricade il centro degli oggetti 
    if bbcenter_x < left_center_bound:
        area = 'sx'
    elif bbcenter_x < center_right_bound:
        area = 'center'
    else:
        area = 'dx'
    
    # Inseriamo l'oggetto nel dizionario adeguato, o se già presente
    # aumentiamo il contatore associato a quell'oggetto
    ob_class = classmap[c]
    if ob_class in objects[area]:
        objects[area][ob_class] += 1
    else:
        objects[area][ob_class] = 1

    rospy.logdebug(f"Found {ob_class}: {p1}, {p2} ({area}); score {s}")

    # nel caso aggiungiamo all'immagine il risultato della detection
    if DEBUG:
        col = (255,0,0) 
        cv2.rectangle(panorama, p1, p2, col, 2)
        p1 = (p1[0]-10, p1[1])
        cv2.putText(panorama, "%s %.2f" % (ob_class,s), p1, cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)


# Generazione della frase:
sentence = "on the left, i see " + objects_sentence(objects['sx']) + ";"
sentence = sentence + " in front of me, i see " + objects_sentence(objects['center']) + ";"
sentence = sentence + " on the right, i see " + objects_sentence(objects['dx']) + ";"

# Il robot parla
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

# Nel caso mostriamo l'immagine con i bounding box della detection,
# e la salviamo su file.
if DEBUG:
    panorama = cv2.line(panorama, (left_center_bound, 0), (left_center_bound, h), (0, 255, 0), 2)
    panorama = cv2.line(panorama, (center_right_bound, 0), (center_right_bound, h), (0, 255, 0), 2)
    cv2.imshow("Panorama", panorama)
    cv2.waitKey(0)
    cv2.imwrite("panorama_od.jpg", panorama)
    cv2.destroyAllWindows()


rospy.loginfo("Task completed. Closing..")