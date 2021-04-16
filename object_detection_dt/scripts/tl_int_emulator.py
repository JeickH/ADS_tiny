#!/usr/bin/env python2


import std_msgs.msg
import random
import math
import copy
import os
import matplotlib.pyplot as plt
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
import rospkg
from action_node.msg import identified_msg
from action_node.msg import scene_msg
from action_node.msg import instruction_msg
from action_node.msg import path_msg
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped, FSMState, BoolStamped







int_path=[]
ind_path=False
scene=''
upd_pose=False
offset=[15,22,2]
last_command=[0,0,0]
actual_pose= np.sum([[8.75,2.5,0],offset],axis=0)
act_emul=False
rospack = rospkg.RosPack()

def save_path(data):
    global ind_path
    global int_path
    global scene
    or_path=data.path
    int_path=np.array_split(or_path, len(or_path)/2)
    ind_path=True
    print(int_path)
    print('path saved: '+ str(ind_path))

def continuecb(data):
    global ind_path
    global int_path
    global scene
    print('LA CDM')

def updated_pose_cb(data):
    global upd_pose
    upd_pose=True
    #print("Continue signal received")



def publish_tl_int():
    global ind_path
    global int_path
    global scene
    global image_pub
    #images=os.listdir(rospack.get_path('object_detection_dt')+"/images/images_tl")
    #print(images)
    for d in sorted(os.listdir(rospack.get_path('object_detection_dt')+"/images/images_tl"))[70:]:
        #print(d)
        path=rospack.get_path('object_detection_dt')+"/images/images_tl/"+d
        image = cv2.imread(path,1)
        image=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        image_pub.publish(bridge.cv2_to_imgmsg(image,"rgb8"))
        #tiempo entre cada imagen
        print("published "+ str(path[50:]))
        time.sleep(0.5)
        #print(path)
    print('PUBLISHING DONE')

def publish_t_int():
    global ind_path
    global int_path
    global scene
    global image_pub


    #images=os.listdir(rospack.get_path('object_detection_dt')+"/images/images_tl")
    #print(images)
    for d in sorted(os.listdir(rospack.get_path('object_detection_dt')+"/images/prueba")):
        #print(d)
        #path=rospack.get_path('object_detection_dt')+"/images/free_int/"+d
        path=rospack.get_path('object_detection_dt')+"/images/prueba/"+d
        image = cv2.imread(path,1)
        #image=cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
        image_pub.publish(bridge.cv2_to_imgmsg(image,"rgb8"))
        #tiempo entre cada imagen
        time.sleep(0.5)
        #print(path)
    print('PUBLISHING DONE')

def reset_obj_det():
    global oedr
    msg45=std_msgs.msg.Bool()
    msg45.data=True
    oedr.publish(msg45)
    print("message Reset OEDR published")

def publish_start_pose():
    global ind_path
    global int_path
    global scene
    global odom_pub
    global upd_pose
    global offset 

    pub_msg=Pose2DStamped()
    #print(pub_msg)
    pub_msg.header.stamp=tm.now()
    pub_msg.x=8.75+offset[0]
    pub_msg.y=2.5+offset[1]
    pub_msg.theta=0+offset[2]
    odom_pub.publish(pub_msg)
    print("published: \n"+ str(pub_msg))



def update_command(data):
    global last_command
    global ind_path
    global int_path
    global scene
    global odom_pub
    global upd_pose
    global offset
    global actual_pose
    global act_emul 

    th=data.omega
    mult=0.2
    pub_msg=Pose2DStamped()
    pub_msg.header.stamp=tm.now()
    pub_msg.header.frame_id="ODOMETRY EMULATOR MESSAGE"
    if act_emul:
            au_y=math.sin(th)*data.v/0.5
            au_x=math.cos(th)*data.v/0.5
            pub_msg.x=mult*au_x+actual_pose[0]
            pub_msg.y=mult*au_y+actual_pose[1]
            pub_msg.theta=th
            odom_pub.publish(pub_msg)
            actual_pose[0]=pub_msg.x
            actual_pose[1]=pub_msg.y
            print("Current Pose updated")

def act_emulate_odom():
    global act_emul
    act_emul=True
    print("ODOMETRY EMULATOR ACTIVATED")

def pb_cmd():
    global cmds
    msg=Twist2DStamped()
    msg.header.frame_id="EMULATOR MESSAGE"
    print("INGERESE LA VELOCIDAD")
    msg.v=float(input())
    print("INGRESE EL ANGULO")
    msg.omega=float(input())
    cmds.publish(msg)
    print("MENSAJE PUBLICADO")
    time.sleep(0.3)
    msg.v=0
    msg.omega=0
    cmds.publish(msg)

def fsm_message():
    msg=FSMState()
    msg.header.stamp=tm.now()
    msg.header.frame_id="EMULATOR MESSAGE"
    print("INGRESE EL MODO:")
    print("1. NORMAL_JOYSTICK_CONTROL")
    print("2. LANE_FOLLOWING")
    #print("")
    mode=int(input())
    if mode==1:
        msg.state="NORMAL_JOYSTICK_CONTROL"
    if mode==2:
        msg.state="LANE_FOLLOWING"
    car_mode.publish(msg)

def main(args):    
    
    try:
        global scene
        global new_inst
        for i in range(20):
            print('----CHOOSE AN OPCION----')
            print('1. Change high level instruction')
            print('2. Publish images and poses emulating car arriving to a tl intersection')
            print('3. Publish images and poses emulating car arriving to a t intersection')
            print('4. Publish car initial pose in the intersection')
            print('5. Activate odometry emulator')
            print('6. Reset OEDR')
            print('7. cmd message')
            print('8. FSM mode message')
            print('9. exit')
            opcion = int(input())
            print("\nCHOOSEN OPTION: " + str(opcion))
            if opcion ==1:
                print('\nINPUT AN INSTRUCTION left(1) forward(2) right(3)]')
                left='left'
                forward='forward'
                right='right'
                instru=input()
                if instru ==1 or instru == 2 or instru == 3:
                    msg1=instruction_msg()
                    msg1.header.stamp=tm.now()
                    msg1.header.frame_id="INSTRUCTION MESAGGE"
                    msg1.header.seq=1
                    if instru==1:
                        msg1.instruction="left"
                    elif instru==2:
                        msg1.instruction="forward"
                    elif instru==3:
                        msg1.instruction="right"
                    new_inst.publish(msg1)
                else:    
                    print("Input a valid instruction")
            elif opcion==2:
                scene='traffic_light'
                publish_tl_int()
            elif opcion==3:
                scene='intersection'
                publish_t_int()
            elif opcion==4:
                publish_start_pose()
            elif opcion==5:
                act_emulate_odom()
            elif opcion==6:
                reset_obj_det()
            elif opcion==7:
                pb_cmd()
            elif opcion==8:
                fsm_message()
            elif opcion==9:
                sys.exit()
            else:
                print('INPUT A NUMBER BETWEEN 1 AND 5')


    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        sys.exit()

rospy.init_node('Intersection_emulator', anonymous=True)
rospy.loginfo("INTERSECTION EMULATOR")
duckie="duckiebot2"
bridge = CvBridge()
image_pub=rospy.Publisher("%s/camera_node/image/raw" % (duckie), Image, queue_size=1)
odom_pub=rospy.Publisher("%s/velocity_to_pose_node/pose" % (duckie),Pose2DStamped, queue_size=1)
new_inst= rospy.Publisher("%s/path_planning/instruction" % (duckie),instruction_msg,queue_size=1)
oedr=rospy.Publisher("%s/object_detection/stop_line_detector" % (duckie),std_msgs.msg.Bool,queue_size=1)
cmds=rospy.Publisher("%s/car_cmd_switch_node/cmd" % (duckie), Twist2DStamped, queue_size=1)
car_mode=rospy.Publisher("%s/fsm_node/mode" % (duckie),FSMState, queue_size=1)


updated_pose_pub=rospy.Subscriber("%s/update_pose" % (duckie),std_msgs.msg.Bool , updated_pose_cb)
#activator_sl_sub=rospy.Subscriber("%s/object_detection/stop_line" % (duckie),std_msgs.msg.Bool,continuecb)    
car_path=rospy.Subscriber("%s/path_planning/path" % (duckie), path_msg,save_path)
commands=rospy.Subscriber("%s/car_cmd_switch_node/cmd" % (duckie), Twist2DStamped, update_command)



if __name__ == '__main__':
    main(sys.argv)