#!/usr/bin/env python2


import std_msgs.msg
from std_msgs.msg import Int32MultiArray,MultiArrayLayout,MultiArrayDimension
import random
import math
import copy
import os
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
#import obstacles
import roslib
import sys
import rospy
import time
from rospy import Time as tm

from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped, FSMState, BoolStamped
#from PIL import Image

"""
def publicador():
    instruction.publish("forward")
    time.sleep(5)
    print("se publico un mensaje")"""
or_path=[]
2dpath=[]

def save_path(data):
    or_path=data
    2dpath=np.array_split(or_path, len(or_path)/2)

def main(args):
    rospy.init_node('EMULATOR POSE 2D', anonymous=True)
    #rospy.loginfo("Publisher Scene")
    #print("Start RRT path planning")
    bridge = CvBridge()
    odom_pub=rospy.Publisher("%s/duckiebot9/velocity_to_pose_node/pose" % (rospy.get_param('~veh',"no_veh")),Pose2DStamped, queue_size=1)
        

    car_path=rospy.Subscriber("/Path_planning/path", Int32MultiArray,save_path)

    #new_instruction= rospy.Subscriber("/Path_planning/instruction",std_msgs.msg.String,new_instruction_callback)

    rospy.loginfo("EMULATOR POSE 2D STARTED")

    cont=0
    #images=[image1,image2]
    
    try:
        while not KeyboardInterrupt:
            if len(or_path) i not 0:    
                #LOGICA KEIRY
                for n in range(2dpath):
                    

                #PUBLICADOR
                pub_msg=Pose2DStamped()
                pub_msg.header.stamp=tm.now()
                pub_msg.pose.x=x
                pub_msg.pose.y=y
                pub_msg.pose.theta=theta
                odom_pub.publish(pub_msg)

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        sys.exit(1)


if __name__ == '__main__':
    main(sys.argv)