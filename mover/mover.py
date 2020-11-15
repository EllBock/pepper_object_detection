#!/usr/bin/env python

import rospy, tty, sys, select
from std_msgs.msg import String
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

movements = {
        'a':[0*almath.TO_RAD, -5*almath.TO_RAD],
        'd':[0*almath.TO_RAD, 5*almath.TO_RAD],
        'w':[5*almath.TO_RAD, 0],
        's':[-5*almath.TO_RAD, 0*almath.TO_RAD],
    }

def getKey():
    rlist, _, _ = select.select([sys.stdin], [], [], 0.3)
    if rlist:
        return sys.stdin.read(1)
 

if __name__ == '__main__':

    try:
        rospy.init_node('mover', anonymous=True)
        p = rospy.Publisher('/pepper_robot/pose/joint_angles', JointAnglesWithSpeed, queue_size=0)
        s = JointAnglesWithSpeed()
        s.joint_names=['HeadPitch', 'HeadYaw']
        s.relative=0
        s.speed=0.2
        rate = rospy.Rate(0.2)
        while(1):
            key = getKey()
            if key in movements.keys():
                s.joint_angles= movements[command]
                rospy.loginfo(s.joint_angles)
                p.publish(s)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
