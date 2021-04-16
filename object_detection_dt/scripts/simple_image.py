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
    rospy.init_node('Image', anonymous=True)
    rospy.loginfo("Publisher Scene")
    #print("Start RRT path planning")
    bridge = CvBridge()
    image_pub=rospy.Publisher("/image_decompressed_topic", Image, queue_size=1)
    #new_instruction= rospy.Subscriber("/Path_planning/instruction",std_msgs.msg.String,new_instruction_callback)
    print("se inicializo el publicador Imagen")
    path1=os.getcwd()+ '/src/object_detection_dt/GaDuckie1070.jpeg'
    image1 = cv2.imread(path1,1)
    image1=cv2.cvtColor(image1,cv2.COLOR_BGR2RGB)
    path2=os.getcwd()+ '/src/object_detection_dt/GaDuckie3787.jpeg'
    image2 = cv2.imread(path2,1)
    image2=cv2.cvtColor(image2,cv2.COLOR_BGR2RGB)
    #print(str(image.size))
    print('se encontro el imagen path')
    #publicar=publicador()
    #rrt = RRT(instruction)
    # ====Search Path with RRT====
    # Set Initial parameters
    #path = rrt.Planning(animation=True)
    #print(str(path))
    # Draw final path
    
          # Need for Mac
        #  plt.show()
    cont=0
    
    try:
        while True:
            if cont%2==0:
                image_pub.publish(bridge.cv2_to_imgmsg(image1,"rgb8"))
                print('imagen 1')
            else:
                image_pub.publish(bridge.cv2_to_imgmsg(image2,"rgb8"))
                print('imagen 2')
            cont +=1
            time.sleep(30)
            print("se publico una imagen")
            print('tiempo de publicacion:')
            print('secs:'+ str(tm.now().secs))
            print('nsecs:'+ str(tm.now().nsecs))
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        sys.exit(1)


if __name__ == '__main__':
    main(sys.argv)
