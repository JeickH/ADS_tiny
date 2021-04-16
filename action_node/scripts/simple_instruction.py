#!/usr/bin/env python2


import std_msgs.msg
import random
import math
import copy
import obstacles
import roslib
import sys
import rospy
import time
from rospy import Time as tm
from action_node.msg import instruction_msg



"""
def publicador():
    instruction.publish("forward")
    time.sleep(5)
    print("se publico un mensaje")"""

def main(args):
    rospy.init_node('Instruction', anonymous=True)
    rospy.loginfo("Publisher Instruction")
    #print("Start RRT path planning")
    instruction=rospy.Publisher("%s/Path_planning/instruction" % (rospy.get_param('~veh',"no_veh")), instruction_msg, queue_size=10)
    #new_instruction= rospy.Subscriber("/Path_planning/instruction",std_msgs.msg.String,new_instruction_callback)
    print("se inicializo el publicador instruction")
    msg=instruction_msg()
    seq=0
    msg.header.frame_id='Instruction'
    try:
        while True:
            seq+=1
            msg.header.stamp=tm.now()
            msg.header.seq=seq
            if seq%2==0:
                msg.instruction='left'
            else:
                msg.instruction='right'
            instruction.publish(msg)
            time.sleep(30)
            print("se publico una instruccion")
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
        sys.exit(1)


if __name__ == '__main__':
    main(sys.argv)
