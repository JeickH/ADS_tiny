#!/usr/bin/env python2

import numpy as np
import std_msgs.msg
import os
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import std_msgs.msg
import random
import math
import copy
import obstacles
import plan_ol
import roslib
import sys
import rospy
import time
import copy
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from rospy import Time as tm
from action_node.srv import path_planning, path_planningResponse, path_planningRequest
from action_node.msg import identified_msg
from action_node.msg import scene_msg
from action_node.msg import instruction_msg
from action_node.msg import path_msg
#from action_node.msg import Twist2DStamped, Pose2DStamped, FSMState, BoolStamped
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped, FSMState, BoolStamped
import roslaunch

class action():

    def visualizer_cb(self,data ):
        self.visualizer=data.data

    def path_planning_client(self):
        #print('entro a metodo cliente')
        #no pasa de aqui
        rospy.loginfo("Waiting for Path Planning Service")
        rospy.wait_for_service('pathPlanning',5)
        try:
            #print('entro al try')
            self.Path_planning_srv1 = rospy.ServiceProxy('pathPlanning', path_planning)
            #print('se llamo el servidor')
            req=path_planningRequest()
            req.instruction=self.instruction
            req.scene=self.scene
            req.objects=self.objects_srv
            #req.animation=True
            req.animation=False
            path=self.Path_planning_srv1(req)
            #print('se llamo el servicio')
            #print(type(path.path))
            return(path.path)
            #print(path)
            self.steps=0
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def new_scene(self, data):
        #print("se recibio una nueva scene")
        self.in_scene= True
        #print(data)
        #print(data.header.stamp.secs-self.secs_curr_scene)
        #if data.header.stamp.secs-self.secs_curr_scene > 45 :
        print('scene: '+ str(data.scene))
        self.scene=data.scene
        self.secs_curr_scene=data.header.stamp.secs
            
        #else:
        #print('no ha cumplido el tiempo minimo para el nuevo scene')
        
    def new_instruction_callback(self, data):
        #print("se recibio una nueva instruccion")
        if data.header.stamp.secs-self.secs_curr_inst > 45 :
            self.instruction=data.instruction
            print("se modifico instruction como "+ self.instruction)
            self.secs_curr_inst=data.header.stamp.secs
            #print("se modifico secs_curr_inst como "+ str(self.secs_curr_inst))
        else:
            print('no ha cumplido el tiempo minimo para la nueva instruccion')
    #parar si se identifica un peaton cerca o un carro en frente 
    def emergency_stop(self):
        obj_list=self.objects_all
        for obj in obj_list:
            lx=obj[5]-obj[3]
            ly=obj[4]-obj[2]
            cx=(obj[5]+obj[3])/2
            cy=(obj[4]+obj[2])/2
            #se identifica que esta cerca por el alto y ancho del box y se identifica que
            #esta en frente por el centro del box
            if obj[1]==1 and (ly>200 or lx>280) and (cx>17 and cx<494 and cy>181 and cy<406):
                print('vehiculo en frente')
                print("Emergency stop alert")
                self.emer_msg.header.stamp=tm.now()
                self.emer_msg.data=True
                self.commands_emer.publish(self.emer_msg)

            elif obj[1]==5 and (ly>85 or lx>90) and (cx>17 and cx<494 and cy>181 and cy<406):
                print('peaton en frente')
                print("Emergency stop alert")
                self.emer_msg.header.stamp=tm.now()
                self.emer_msg.data=True
                self.commands_emer.publish(self.emer_msg)

            else:
                self.emer_msg.header.stamp=tm.now()
                self.emer_msg.data=False
                self.commands_emer.publish(self.emer_msg)

            if self.emer_msg.data==False:
                print('no emergency stop')

    def planning_stop_method(self, obj_list):
        cont=0
        for obj in obj_list:
            #print('obj[1]==6 and ly>=65 '+ str(obj[1]==6 and ly>=65))
            #print('tm.now().secs-self.secs_curr_stop>40 '+ str(tm.now().secs-self.secs_curr_stop>40))
            #definiendo que la longitud en y sea mayor a 65 pixeles
            #me aseguro que el vehiculo este en la posicion justo en frente de la linea de pare
            if obj[1]==6 and tm.now().secs-self.secs_curr_stop>30:
                #START ADS level2 commands
                #ind=1
                #self.activate_nodes(ind)
                #self.planning_stop=True
                #print('planning_stop set as True')
                self.curr_miny_pare=obj[2]
                self.secs_curr_stop=self.id_time
                #self.commands.publish(self.stop_msg)

    #object raw= [seguridad, clase, ymin, xmin,ymax,xmax]
    def set_ymin_pare(self, obj_list):
        for obj in obj_list:
            if obj[1]==6 and tm.now().secs-self.secs_curr_stop>30:
                
                self.curr_miny_pare=obj[2]
                self.secs_curr_stop=self.id_time
        
    def set_vehicles_in_map(self,objects_raw):
        cars_id=[]
        print("minypare"+str(self.curr_miny_pare))
        self.zones_vehicles=[0,0,0,0,0,0,0,0,0,0,0,0,0]
        #print('min y pare ' +str(self.curr_miny_pare) )
        for i in range(len(objects_raw)):
            if objects_raw[i][1]==1:
                curr_car=np.array((0,0,0), dtype=float)
                #coordenada en x del centro del box 
                cx=(objects_raw[i][5]+objects_raw[i][3])/2
                #definir posicion en Y de los vehiculos
                if self.curr_miny_pare-objects_raw[i][4]>=135:
                    curr_car[1]=11.25
                if self.curr_miny_pare-objects_raw[i][4]>=80 and self.curr_miny_pare-objects_raw[i][4]<135:
                    curr_car[1]=8.75
                if self.curr_miny_pare-objects_raw[i][4]<80:
                    curr_car[1]=6.25
                #definir posicion en X de los vehiculos
                #zona 1
                if cx<70 and curr_car[1]==8.75:
                    curr_car[0]=3
                    self.zones_vehicles[0]=1
                    #print('vehicle in zone 1')
                #zona 2
                if cx>=70 and cx<150  and curr_car[1]==8.75:
                    curr_car[0]=4.3
                    self.zones_vehicles[1]=1

                    #print('vehicle in zone 2')
                #zona 3
                if cx>=150 and cx<260  and curr_car[1]==8.75:
                    curr_car[0]=6
                    self.zones_vehicles[2]=1
                    #print('vehicle in zone 3')
                #zona 4
                if cx>=260 and cx<420  and curr_car[1]==8.75:
                    curr_car[0]=9
                    self.zones_vehicles[3]=1
                    #print('vehicle in zone 4')
                #zona 5
                if cx>=420 and cx<540  and curr_car[1]==8.75:
                    curr_car[0]=11
                    self.zones_vehicles[4]=1
                    #print('vehicle in zone 5')
                #zona 6
                if cx>=540 and cx<=640  and curr_car[1]==8.75:
                    curr_car[0]=13
                    self.zones_vehicles[5]=1
                    #print('vehicle in zone 6')
                #zona 7
                if cx<70 and curr_car[1]==6.25:
                    curr_car[0]=4.3
                    self.zones_vehicles[6]=1
                    #print('vehicle in zone 7')
                #zona 8
                if cx>=70 and cx<220  and curr_car[1]==6.25:
                    curr_car[0]=6
                    self.zones_vehicles[7]=1
                    #print('vehicle in zone 8')
                #zona 9
                if cx>=220 and cx<480  and curr_car[1]==6.25:
                    curr_car[0]=9
                    self.zones_vehicles[8]=1
                    #print('vehicle in zone 9')
                #zona 10
                if cx>=480  and curr_car[1]==6.25:
                    curr_car[0]=11
                    self.zones_vehicles[9]=1
                    #print('vehicle in zone 10')
                #zona 11
                if cx>=140 and cx<280  and curr_car[1]==11.25:
                    curr_car[0]=6
                    self.zones_vehicles[10]=1
                    #print('vehicle in zone 11')
                #zona 12
                if cx>=280 and cx<420 and curr_car[1]==11.25:
                    curr_car[0]=9
                    self.zones_vehicles[11]=1
                    #print('vehicle in zone 12')
                #tamano de obstaculo en mapa 2D
                #print('curr_car')
                #print(curr_car)
                curr_car[2]=1
                cars_id.append(curr_car)
        #cars_id es una lista de arrays con la posicion de los vehiculos
        #Serializar id_cars para enviarlo a el servicio como una lista y no como lista de arrays
        for i in range(len(cars_id)):
            self.objects_srv.append(cars_id[i][0])
            self.objects_srv.append(cars_id[i][1])
            self.objects_srv.append(cars_id[i][2])

        #self.objects_srv=np.frombuffer(cars_id.tobytes(),'float32')
        #rospy.loginfo("Action Node Message")
        #print('vehicles positions')
        #print(cars_id)
        #print('objects_srv')
        #print(self.objects_srv)
    #determinar si se puede o no ejecutar el path con base en objects_srv
    #la bandera es self.execute_path
    def ask_to_execute_path(self):
        print('zones_vehicles')
        print(self.zones_vehicles)
        classes2=[]
        green_light=False
        if self.execute_path==False:
            for obj in self.objects_all:
                    classes2.append(int(obj[1]))
            #print(classes2)
            if self.scene=='traffic_light':
                if self.instruction=='left':
                    self.instruction='forward'
                    print('High level instruction switched to forward due to traffic rules')
                for cl in classes2:
                    if cl ==2 or cl==3:
                        green_light=False
                    elif cl ==4:
                        green_light=True
                if green_light and self.instruction=='forward' and self.zones_vehicles[3]==0 and self.zones_vehicles[7]==0 and self.zones_vehicles[8]==0:
                    #print(2)
                    self.execute_path=True
                elif green_light and self.instruction=='right' and self.zones_vehicles[7]==0 and self.zones_vehicles[8]==0 and self.zones_vehicles[9]==0:
                    #print(3)
                    self.execute_path=True
                else :
                    #print(4)
                    self.execute_path=False

            if self.scene=='intersection':
                if self.instruction=='forward':
                    self.instruction='right'
                    print('High level instruction switched to right due to traffic rules')
                if self.instruction=='right':
                    if self.zones_vehicles[6]==0 and self.zones_vehicles[7]==0 and self.zones_vehicles[8]==0 and self.zones_vehicles[9]==0:
                        self.execute_path=True
                    else:
                        self.execute_path=False
                if self.instruction=='left':
                    if self.zones_vehicles[1]==0 and self.zones_vehicles[2]==0 and self.zones_vehicles[3]==0 and self.zones_vehicles[4]==0 and self.zones_vehicles[5]==0 and self.zones_vehicles[6]==0 and self.zones_vehicles[7]==0 and self.zones_vehicles[8]==0:
                        self.execute_path=True
                    else:
                        self.execute_path=False
        else:
            print('Already executing a motion planning')

    #metodo para activar y desactivar los nodos
    def activate_nodes(self, num_case):
        #si ind es 0 encender el seguidor de carril y desactivar el ADS   
        #si ind es 1 encender el ADS y desactivar seguidor de carril
        msg=FSMState()
        msg.header.stamp=tm.now()
        msg.header.frame_id="INTERSECTION ADS CONTROLLED"
        if num_case == 1:
            msg.state="NORMAL_JOYSTICK_CONTROL"
            self.car_mode.publish(msg) 
            #rospy.loginfo("Action Node Message")
            print('SWITCHING TO CONTROL BY ADS')
        if num_case ==0:
            msg.state="LANE_FOLLOWING"
            self.car_mode.publish(msg)
            #rospy.loginfo("Action Node Message")
            print('SWITCHING TO CONTROL BY LANE FOLLOWING MODULE')

    def update_pose(self,data):
        #self.poseSec=data.header.stamp.secs
        self.current_pose[0]=data.x
        self.current_pose[1]=data.y
        self.current_pose[2]=data.theta
        print("pose updated:"+str(self.current_pose) + "con seq: "+ str(data.header.seq) +"y stamp: "+str(data.header.stamp))
        self.updated_pose=True
        msg=std_msgs.msg.Bool()
        msg.data=True
        self.updated_pose_pub.publish(msg)

    #metodo para enviar los comandos a las ruedas para que ejecuten el path

    def cmd_execute_path(self):
        
        #cont_pose=1
        while self.running_path:
            #Obtener posicion incial
            #para cada punto del path, definir el angulo y la velocidad del duckie
            print(self.execute_cl)
            final_goal=self.real_path[len(self.real_path)-1]*1.1/5
            if self.execute_cl:
                print("ENTRO A CL")
                curr_goal=self.real_path[self.ind_path]*1.1/5
                final_goal=self.real_path[len(self.real_path)-1]*1.1/5
                if self.updated_pose:
                    #------------START CLOSE LOOP METHOD--------
                    print("initial pose:"+str(self.init_pose))
                    print("current_pose:"+str(self.current_pose))
                    #AJUSTE DE LA GANANCIA PARA EL ADS DE 7.8825
                    self.rel_pose[0]=self.current_pose[0]-self.init_pose[0]
                    self.rel_pose[0]=self.rel_pose[0]+8.75*1.1/5
                    #self.rel_pose[0]=self.current_pose[0]-23.75+8.75
                    #AJUSTE DE LA GANANCIA PARA EL ADS DE 7.8825
                    self.rel_pose[1]=self.current_pose[1]-self.init_pose[1]
                    self.rel_pose[1]=self.rel_pose[1]+2.5*1.1/5
                    #self.rel_pose[1]=self.current_pose[1]-24.5+2.5
                    self.rel_pose[2]=self.current_pose[2]-self.init_pose[2]
                    #self.rel_pose[2]=self.rel_pose[2]+math.pi/2
                    self.rel_pose[2]=(self.rel_pose[2]%(2*math.pi))+math.pi/2
                    #self.rel_pose[2]=self.current_pose[2]-2+math.pi/2
                    #rel_pose=np.sum([np.subtract(self.current_pose,self.init_pose),[8.75,2.5,math.pi/2]],axis=0)
                    print('rel_pose'+str(self.rel_pose))
                    print("curr_goal"+str(curr_goal))
                    #atan2(y/x) return a value between pi and -pi

                    #el omega del mensaje es el angulo relativo a desplazarse del duckie
                    theta=math.atan2(curr_goal[0,1]-self.rel_pose[1],curr_goal[0,0]-self.rel_pose[0])
                    print("atan2 result:"+str(theta))
                    theta= (theta+2*math.pi)%(2*math.pi)
                    print("mod result:"+str(theta))
                    theta=theta-self.rel_pose[2]
                    print("ajust to relative ang result:"+str(theta))
                    if theta>math.pi:
                        theta=theta%(2*math.pi)
                    elif theta<(-math.pi):
                        theta=theta%(2*math.pi)
                    #theta=(pi/2)-theta
                    #theta_msg=theta-self.current_pose[2]
                    cmd_msg=Twist2DStamped()
                    cmd_msg.header.stamp=tm.now()
                    cmd_msg.header.seq=self.steps
                    cmd_msg.header.frame_id='cmd generated by ADS'
                    #velocidad constante
                    #cmd_msg.v=0.5
                    #velocidad variable proporcional a el angulo theta
                    delta_th=abs(theta-self.past_theta)
                    #angulo mayor a 30 grados o 0.52 radianes
                    #if delta_th>0.52:
                     #   cmd_msg.v=-0.05208*delta_th+0.527
                    #else:
                        #cmd_msg.v=0.2
                    self.emer_msg.header.stamp=tm.now()
                    self.emer_msg.data=False
                    self.commands_emer.publish(self.emer_msg)

                    cmd_msg.v=0.3
                    cmd_msg.omega=theta*1.3

                    self.commands.publish(cmd_msg)
                    print("en execute se publico la 1")
                    print(cmd_msg)
                    self.steps+=1
                    time.sleep(0.3)
                    #self.stop_msg.header.stamp=tm.now()
                    #--pare despues de cada mensaje publicado
                    cmd_msg2=Twist2DStamped()
                    cmd_msg2.header.stamp=tm.now()
                    cmd_msg2.v=0
                    cmd_msg2.omega=0
                    cmd_msg2.header.frame_id='cmd generated by ADS'
                    self.commands.publish(cmd_msg2)
                    print("en execute se publico la 2")
                    self.updated_pose=False
                #self.emer_msg.header.stamp=tm.now()
                #self.emer_msg.data=True
                #self.commands_emer.publish(self.emer_msg)
                #----------------------------------------
                #print('se publico un cmd generated by ADS')
                #print(cmd_msg)
                #self.past_theta=theta
                #pausa para actualizar posicion
                #time.sleep(1)
                #print("diferencia de curr_goal y pose rel:" +str(abs(self.rel_pose[0]-curr_goal[0][0]))+" "+str(abs(self.rel_pose[1]-curr_goal[0][1])))
                if abs(self.rel_pose[0]-curr_goal[0][0])<0.1 and abs(self.rel_pose[1]-curr_goal[0][1])<0.1 :
                    self.ind_path+=1
                    print('cambio de curr_goal')
                #self.updated_pose=False
                #print('update_pose False')
            #----------END CLOSE LOOP METHOD------------

            #----------START OPEN LOOP METHOD-----------
            else:
                #print("ENTRO A OL")
                #print(self.execute_cl)
                cmd_msg2=Twist2DStamped()
                end_ol=False
                if self.instruction=="left" and self.scene=="traffic_light":
                    self.instruction=="right"
                if self.instruction=="forward" and self.scene=="intersection":
                    self.instruction=="right"
                if self.instruction=="right":
                    planning_ol=plan_ol.right
                if self.instruction=="left":
                    planning_ol=plan_ol.left
                if self.instruction=="forward":
                   planning_ol=plan_ol.forward
                for pl in range(len(planning_ol)):
                    cmd_msg2.header.stamp=tm.now()
                    cmd_msg2.v=planning_ol[pl][0]
                    cmd_msg2.omega=planning_ol[pl][1]
                    cmd_msg2.header.frame_id='cmd generated by ADS'
                    self.commands.publish(cmd_msg2)
                    #print(cmd_msg2)
                    time.sleep(0.3)
                    cmd_msg2.header.stamp=tm.now()
                    cmd_msg2.v=0
                    cmd_msg2.omega=0
                    cmd_msg2.header.frame_id='cmd generated by ADS'  
                    self.commands.publish(cmd_msg2)
                    #probe delay
                    #time.sleep(3)
                end_ol=True  
            #mirar si la condicion para que este muy cercano al nodo final esta bien de sintaxis
            if abs((self.rel_pose[0]-final_goal[0][0])<0.1 and abs(self.rel_pose[1]-final_goal[0][1])<0.1) or end_ol:
                print('INTERSECTION DONE')
                self.execute_path=False
                ind=1
                self.activate_nodes(ind)
                self.in_scene=False
                self.path_ready=False
                self.steps=0
                self.ind_path=1
                self.running_path=False
                self.first=True
                end_ol=False

    def stop_linecb(self, data):
        if data.data== True and tm.now().secs-self.secs_curr_stop>15:
            ind=1
            self.activate_nodes(ind)
            self.planning_stop=True
            self.stop_msg.header.stamp=tm.now()
            self.stop_msg.omega=0
            self.commands.publish(self.stop_msg)
            #self.lacdm=self.current_pose
            print('planning_stop set as True')


    def identified_msg_callback(self, data):
        #borrar datos de carros despues de 15 segunds sin recibir mensajes
        
        if data.header.stamp.secs-self.secs_curr_obj>15:
            self.objects_srv=[]
        objects_ser=data.data
        self.id_time=data.header.stamp.secs
        i=0
        #des-serializar el mensaje recibido en una lista de listas
        N = 6
        objects_raw = [objects_ser[n:n+N] for n in range(0, len(objects_ser), N)]
        #print('objects_raw:')
        #print(objects_raw)
        self.objects_all=objects_raw
        self.curr_miny_pare=315
        #self.secs_curr_stop=self.id_time
        #parada de emergencia ante un carro en frente y ante un peaton en frente
        self.emergency_stop()
        #print('ejecuto el proceso de emergency stop')
        self.set_ymin_pare(objects_raw)

        
        #ESTE PROCESO DEBERIA ARROJAR EL PLANNING STOP SIEMPRE TRUE Y ENVIAR EL MENSAJE DE STOP A LAS RUEDAS
        #if self.planning_stop == False and self.execute_path == False and self.path_ready==False and self.in_scene==True:
            #identificar si estamos ante una linea de pare y si si, detenerse, este proceso se 
            #ejecuta solo cuando no esta haciendo el pare y planeando el camino y no esta ejecutando un camino 
        #    self.planning_stop_method(objects_raw)
        #    print('ejecuto el proceso de planning stop')

        #hacer el llamado al servicio solo si se detuvo en la linea de pare
        #identificar vehiculos en la interseccion
        if self.running_path==False and self.execute_path == False and self.in_scene==True and self.running_path==False:
            self.set_vehicles_in_map(objects_raw)
            print('Set vehicles on 2D map process executed')

        #si realiza el pare ante la interseccion hace el planning
        if self.running_path==False and self.planning_stop==True and self.in_scene==True and self.path_ready==False and self.execute_path==False:
            ser_path=self.path_planning_client()
            #print('se hizo el llamado cliente')
            #print(ser_path)
            self.real_path=[]
            ser_path2=[]
            #print(type(ser_path[2]))
            #for tup in range(len(ser_path)-1):
                #ser_path[tup]=round(ser_path[tup])
            #self.real_path=np.zeros((len(ser_path)/2,2))
            #print((len(ser_path)/2))
            #print(range((len(ser_path)/2)))
            for x in range((len(ser_path)/2)):
                curr_node=np.zeros((1,2))
                i=2*x+1 #1-3-5-7-9
                y=2*x   #0-2-4-6-8
                #print(ser_path[y])
                #print(ser_path[i])
                curr_node[0,0]=round(ser_path[y],2)
                curr_node[0,1]=round(ser_path[i],2)
                ser_path2.append(curr_node[0,0]*0.1)
                ser_path2.append(curr_node[0,1]*0.1)
                self.real_path.append(curr_node)
                #real_path[x][0]=ser_path[y]
                #real_path[x][1]=ser_path[i]

            #print('new path:')
            #print(self.real_path)
            path_msg_pub=path_msg()
            path_msg_pub.header.stamp=tm.now()
            path_msg_pub.header.frame_id='PATH PLANNING MESSAGE'
            path_msg_pub.header.seq=self.steps
            path_msg_pub.path=ser_path2
            #print(ser_path2)
            self.path_pub.publish(path_msg_pub)
            self.planning_stop=False
            self.path_ready=True
            #print('planning_stop is set False')

        if self.path_ready==True and self.in_scene==True and self.execute_path==False and self.running_path==False:
            self.ask_to_execute_path()
            if self.execute_path==False:
                msg45=std_msgs.msg.Bool()
                msg45.data=True
                self.first=True
                self.objects_sl_act.publish(msg45)
                #print('se publico un mensaje para activar el detector de objetos')
            print('request to execute: ' + str(self.execute_path))
            
            #self.init_pose=self.current_pose
            #print("waiting 5 seconds")
            #time.sleep(5)

        if self.execute_path==True and self.path_ready==True and self. in_scene==True and self.running_path==False:
            self.running_path=True
            #guardar primera pose
            if self.first:
                self.init_pose[0]=copy.copy(self.current_pose[0])
                self.init_pose[1]=copy.copy(self.current_pose[1])
                self.init_pose[2]=copy.copy(self.current_pose[2])
                self.first=False
                print("GUARDO LA PRIMERA POSE")
            self.cmd_execute_path()
            #print('se realizo el metodo execute path')
            #activar el detector de linea de pare
            msg45=std_msgs.msg.Bool()
            msg45.data=True
            self.objects_sl_act.publish(msg45)

#-------------------------NUEVO--------------------------------

#--------------------------------------------------------------

        
    def __init__(self):
         
         #self.objects_img=rospy.Publisher("/Object_detection/image", Image, queue_size=3)
        self.secs_curr_inst=0
        self.secs_curr_scene=0
        self.secs_curr_obj=0
        self.curr_path=[]
        self.objects_all=[]
        self.objects_srv=[]
        self.planning_stop=False
        self.updated_pose=False
        self.poseSec=0
        self.stop_msg=Twist2DStamped()
        #self.stop_msg.header.stamp=tm.now()
        self.stop_msg.v=0
        #self.stop_msg.omega=0
        self.stop_msg.header.frame_id='STOP MESSAGE'
        self.id_time=tm(0,0)
        self.execute_path=False
        self.steps=0
        self.instruction='left'
        self.scene='intersection'
        self.emer_msg=BoolStamped()
        self.emer_msg.header.frame_id="Emergency Stop"
        self.emer_msg.data=False
        #pose 2D (x,y,theta)
        self.current_pose=[0,0,0]
        self.rel_pose=[0,0,0]
        self.first=True
        #self.pose2_init=[0,0,0]
        self.zones_vehicles=[0,0,0,0,0,0,0,0,0,0,0,0]
        self.in_scene=False
        self.path_ready=False
        self.secs_curr_stop=0
        self.running_path=False
        self.ind_path=1
        self.past_theta=0
        self.init_pose=[0,0,0]
        self.execute_cl=False

        #self.bridge = CvBridge()
        self.duckie='duckiebot2'
        self.new_scene= rospy.Subscriber("%s/path_planning/scene" % (self.duckie),scene_msg,self.new_scene)
        self.new_inst= rospy.Subscriber("%s/path_planning/instruction" % (self.duckie),instruction_msg,self.new_instruction_callback)
        #corregir topic 
        self.new_pose= rospy.Subscriber("%s/velocity_to_pose_node/pose" % (self.duckie),Pose2DStamped,self.update_pose)
        self.objects_id=rospy.Subscriber("%s/object_detection/identified" % (self.duckie),identified_msg ,self.identified_msg_callback)
        self.visualizer_sub=rospy.Subscriber("%s/ads/visualizer" % (self.duckie),std_msgs.msg.Bool,self.visualizer_cb)
        self.activator_sl_sub=rospy.Subscriber("%s/object_detection/stop_line" % (self.duckie),std_msgs.msg.Bool,self.stop_linecb)
        #Los comandos del ADS se publican en el topic ADS_commands 
        self.car_mode=rospy.Publisher("%s/fsm_node/mode" % (self.duckie),FSMState, queue_size=1)
        self.commands=rospy.Publisher("%s/car_cmd_switch_node/cmd" % (self.duckie), Twist2DStamped, queue_size=1)
        self.commands_emer = rospy.Publisher("%s/wheels_driver_node/emergency_stop" % (self.duckie),BoolStamped,queue_size=1)
        self.path_pub=rospy.Publisher("%s/path_planning/path" % (self.duckie),path_msg, queue_size=1)
        self.objects_sl_act=rospy.Publisher("%s/object_detection/stop_line_detector" % (self.duckie),std_msgs.msg.Bool , queue_size=1)
        self.updated_pose_pub=rospy.Publisher("%s/update_pose" % (self.duckie),std_msgs.msg.Bool , queue_size=1)
        

        rospy.loginfo("Subs/pubs Action node Initialized")
        #print("se inicializaron los publicadores y los supscriptores")

def main(args):
    rospy.init_node('Action_node', anonymous=False)
    rospy.loginfo("Start Action node")
    #print("Start Action node")
    act = action()
    #rospy.on_shutdown(node.on_shutdown)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main(sys.argv)