#!/usr/bin/env python2


import std_msgs.msg
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
#from PIL import Image

"""
def publicador():
    instruction.publish("forward")
    time.sleep(5)
    print("se publico un mensaje")"""

def main(args):
    rospy.init_node('CLOSE_PEDESTRIAN', anonymous=True)
    #rospy.loginfo("Publisher Scene")
    #print("Start RRT path planning")
    bridge = CvBridge()
    image_pub=rospy.Publisher("/image_decompressed_topic", Image, queue_size=1)
    #new_instruction= rospy.Subscriber("/Path_planning/instruction",std_msgs.msg.String,new_instruction_callback)
    rospy.loginfo("EMULATOR CLOSE PEDESTRIAN STARTED")
    path1=os.getcwd()+ '/src/object_detection_dt/GaDuckie1348.jpeg'
    image1 = cv2.imread(path1,1)
    image1=cv2.cvtColor(image1,cv2.COLOR_BGR2RGB)
    path2=os.getcwd()+ '/src/object_detection_dt/GaDuckie1365.jpeg'
    image2 = cv2.imread(path2,1)
    image2=cv2.cvtColor(image2,cv2.COLOR_BGR2RGB)
    path3=os.getcwd()+ '/src/object_detection_dt/GaDuckie1390.jpeg'
    image3 = cv2.imread(path3,1)
    image3=cv2.cvtColor(image3,cv2.COLOR_BGR2RGB)
    #print(str(image.size))
    print('Found image paths')

    cont=0
    images=[image1,image2, image3]
    
    try:
        while not KeyboardInterrupt:
            image_pub.publish(bridge.cv2_to_imgmsg(images[cont],"rgb8"))
            print('IMAGE '+str(cont)+' PUBLISHED')
            cont +=1
            time.sleep(2)
            print('tiempo de publicacion:')
            print('secs:'+ str(tm.now().secs))
            print('nsecs:'+ str(tm.now().nsecs))
            if cont >=3:
                cont=0
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        sys.exit(1)


if __name__ == '__main__':
    main(sys.argv)