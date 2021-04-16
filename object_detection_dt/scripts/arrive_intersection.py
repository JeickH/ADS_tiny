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
conti=True
def continuecb(data):
    print("se cambio conti a " + str(not data.data))
    if data.data == True:
        conti=False
    elif data.data==False:
        conti=True


def main(args):
    rospy.init_node('arrive_intersection_emulator', anonymous=True)
    #rospy.loginfo("Publisher Scene")
    #print("Start RRT path planning")
    duckie="duckiebot5"
    bridge = CvBridge()
    image_pub=rospy.Publisher("/duckiebot5/camera_node/image/raw", Image, queue_size=1)
    activator_sl_sub=rospy.Subscriber("%s/object_detection/stop_line" % (duckie),std_msgs.msg.Bool,continuecb)
    commands=rospy.Subscriber("%s/car_cmd_switch_node/cmd" % (duckie), Twist2DStamped, update_command)

    #new_instruction= rospy.Subscriber("/Path_planning/instruction",std_msgs.msg.String,new_instruction_callback)
    rospy.loginfo("EMULATOR CLOSE PEDESTRIAN STARTED")
    path1=os.getcwd()+ '/src/object_detection_dt/images/duckie1067.jpeg'
    image1 = cv2.imread(path1,1)
    image1=cv2.cvtColor(image1,cv2.COLOR_BGR2RGB)
    path2=os.getcwd()+ '/src/object_detection_dt/images/GaDuckie1070.jpeg'
    image2 = cv2.imread(path2,1)
    image2=cv2.cvtColor(image2,cv2.COLOR_BGR2RGB)
    path3=os.getcwd()+ '/src/object_detection_dt/images/duckie77g.jpeg'
    image3 = cv2.imread(path3,1)
    image3=cv2.cvtColor(image3,cv2.COLOR_BGR2RGB)
    path4=os.getcwd()+ '/src/object_detection_dt/images/GaDuckie1073.jpeg'
    image4 = cv2.imread(path4,1)
    image4=cv2.cvtColor(image4,cv2.COLOR_BGR2RGB)
    path5=os.getcwd()+ '/src/object_detection_dt/images/GaDuckie1077.jpeg'
    image5 = cv2.imread(path5,1)
    image5=cv2.cvtColor(image5,cv2.COLOR_BGR2RGB)
    #print(str(image.size))
    print('Found image paths')

    cont=0
    images=[image1,image2, image3, image4, image5]
    
    try:
        for i in range(1000):
            #print(conti)
            time.sleep(3)
            print(conti)
            if conti==True:
                image_pub.publish(bridge.cv2_to_imgmsg(images[cont],"rgb8"))
                print('IMAGE '+str(cont+1)+' PUBLISHED')
                cont +=1
                print('tiempo de publicacion:')
                print('secs:'+ str(tm.now().secs))
                print('nsecs:'+ str(tm.now().nsecs))
                if cont >=5:
                    cont=0
            if conti==False:
                image_pub.publish(bridge.cv2_to_imgmsg(images[2],"rgb8"))
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        sys.exit()


if __name__ == '__main__':
    main(sys.argv)