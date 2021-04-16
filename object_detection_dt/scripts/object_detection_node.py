#!/usr/bin/env python2

import numpy as np
import std_msgs.msg
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
import roslib
import sys
import rospy
from rospy import Time as tm
from object_detection_dt.msg import identified_msg
from object_detection_dt.msg import scene_msg
#import tarfile
import tensorflow as tf
#import zipfile
import json
import time
#import glob
from io import StringIO
import matplotlib.pyplot as plt
from utils import visualization_utils as vis_util
from utils import label_map_util
import rospkg

from colorcorrect import algorithm as cca
from colorcorrect.util import from_pil, to_pil
import PIL 
import copy
import math


from pylab import array, plot, show, axis, arange, figure, uint8, uint32

#sys.path.append('custom_ws/custom_ws/src/action_node/scripts')
#sys.path.append('/usr/local/bin')
#from cv_bridge.boost.cv_bridge_boost import getCvType

class obj_detection():

    def deteccion(self, img_np):
        data=img_np
        #image_np=cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
        rospack = rospkg.RosPack()
        print('Object detection activated')
        print('Tiempo en el que obj_det recibe la imagen: secs: ' + str(tm.now().secs)+ " nsecs:"+ str(tm.now().nsecs))
        #print(rospack.get_path('object_detection_dt'))
        #image_np =self.bridge.imgmsg_to_cv2(data)
        #image_np_expanded=self.image_full
        #data2 = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)


        #PIL_image = PIL.Image.fromarray(np.uint8(data))
        #image_np_org2=to_pil(cca.automatic_color_equalization(from_pil(PIL_image)))
        image_np_org3 = np.asarray(data)
        
        #NEW METHOD
        #maxIntensity = 150 # depends on dtype of image data
        #x = arange(maxIntensity) 
        #cv2_imshow(image)
        # Parameters for manipulating image data
        #phi = 1.9
        #theta = 1.9
        # Increase intensity such that
        # dark pixels become much brighter, 
        # bright pixels become slightly bright
        #newImage0 = (maxIntensity/phi)*(image/(maxIntensity/theta))**0.2
        #image_np_org3 = array(newImage0,dtype=uint8)


        # convert img to gray
        #converter = PIL.ImageEnhance.Color(PIL_image)
        #img2 = converter.enhance(1.4)
        #gray = cv2.cvtColor(np.array(img2), cv2.COLOR_BGR2GRAY)

        # compute gamma = log(mid*255)/log(mean)
        #mid = 0.25
        #mean = np.mean(gray)
        #gamma = math.log(mid*255)/math.log(mean)
        

        # do gamma correction
        #img_gamma1 = np.power(img2, gamma).clip(0,255).astype(np.uint8)
        

        self.image_full=image_np_org3
        print("tiempo hasta equalizacion: secs "+ str(tm.now().secs)+" nsecs: "+str(tm.now().nsecs))

        if self.visualizer: 
            image_np_expanded2 = np.expand_dims(self.image_full, axis=0)
            plt.imshow(self.image_full)
            plt.pause(4)
            plt.close('all')

        #print("before sess")
        with self.detection_graph.as_default():
            with tf.Session(graph=self.detection_graph) as self.sess:
                self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
                self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
                self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
                self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
                self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

                (boxes, scores, classes, num) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections], feed_dict={self.image_tensor: image_np_expanded2})

        PATH_TO_LABELS=rospack.get_path('object_detection_dt')+'/scripts/configuracion/label_map.pbtxt'
        

        #PATH_TO_LABELS=os.getcwd()+'/src/object_detection_dt/scripts/configuracion/label_map.pbtxt'
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=sys.maxsize, use_display_name=True)
        CATEGORY_INDEX = label_map_util.create_category_index(categories)
        
        image_np_edit=vis_util.visualize_boxes_and_labels_on_image_array(
            cv2.cvtColor(self.image_full, cv2.COLOR_BGR2RGB),
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            CATEGORY_INDEX,
            min_score_thresh=0.05,
            use_normalized_coordinates=True,
            line_thickness=3)
        image_np_edit=cv2.cvtColor(image_np_edit,cv2.COLOR_BGR2RGB)
        image_np_msg=self.bridge.cv2_to_imgmsg(image_np_edit,"bgr8")
        #print("se hizo la deteccion de objetos")
        #print(scores)
        #print(classes)
        #print("num detections")
        #print(self.num_detections)
        #cv2.imwrite('/home/jeick/Pictures/output.jpeg', image_np_edit)
        if self.visualizer:
            self.objects_img.publish(image_np_msg)
            plt.imshow(image_np_edit)
            plt.pause(5)
            plt.close('all')
        #EDITAR Y PUBLICAR LOS BOXES Y LA INFORMACION NECESARIA
        boxes_c=[]
        scores_c=[]
        classes_c=[]
        #filtro de score
        for i in range(scores.size):
            if scores[0][i]>0.052:
                #print(scores[0][i])
                boxes_c.append(boxes[0][i])
                scores_c.append(scores[0][i])
                classes_c.append(classes[0][i])
        #desnormalizacion de boxes
        #print(scores_c)
        #print(classes_c)
        #print(boxes_c)
        for box in boxes_c:
            box[0] =round(box[0]*480)
            box[1] =round(box[1]*640)
            box[2] =round(box[2]*480)
            box[3] =round(box[3]*640)

        ind_intersection=False
        which_intersection=''
        ind_int_t=False
        ind_tl=False
        for clas1 in classes_c:
            if clas1==8 or clas1==2 or clas1==3 or clas1==4 or clas1==9:
                ind_intersection=True
                if clas1==8:
                    which_intersection='intersection'
                    ind_int_t=True
                elif clas1==2 or clas1==3 or clas1==4 or clas1==9:
                    which_intersection='traffic_light'
                    ind_tl=True
            if ind_int_t and ind_tl:
                which_intersection='intersection'
        if ind_intersection:
            sc_msg=scene_msg()
            sc_msg.header= std_msgs.msg.Header()
            sc_msg.header.seq=self.seq_sce
            sc_msg.header.stamp=tm.now()
            sc_msg.header.frame_id="Scene"
            sc_msg.scene=which_intersection
            self.scene.publish(sc_msg)
            self.seq_sce+=1
            


            boxes_c2=[]
            scores_c2=[]
            classes_c2=[]
            #FILTROS
            #ZONA OPTIMA --> (YMIN,XMIN,YMAX,XMAX)
            #TAMANO MINMO DE BOX --> (distX,distY)
            #PROPORCION --> distY/distX
            #-----------------
            #FILTRO PARA VEHICULO (clase 1)
            #ZONA OPTIMA: (126,0,456,640)
            #TAMANO MINIMO DE BOX:(100,60)
            #PROPRCION: N/A
            #-----------------
            #FILTRO PARA SEMAFORO ROJO, VERDE Y AMARILLO (clases 2,3 y 4)
            #ZONA OPTIMA: (46,157,215,414)
            #TAMANO MINIMO DE BOX: (42,42)
            #PROPORCION: ENTRE 0.71 Y 1.42 (cuadrada)
            #-----------------
            #FILTRO PARA PEATON (clase 5)
            #ZONA OPTIMA: (240,11,468,636)
            #TAMANO MINIMO DE BOX: (50,50) --> el real era 80x80 
            #pero se redujo pensando en los peatones no adultos
            #PROPORCION: ENTRE 0.5 Y 2
            #-----------------         
            #FILTRO PARA LINEA DE PARE (clase 6)
            #ZONA OPTIMA: (244,38,407,542)
            #TAMANO MINIMO DE BOX:(270,40)
            #PROPORCION: MENOR A 0.3 
            #-----------------
            #FILTRO PARA AVISOS (clases 7, 8 y 9)
            #ZONA OPTIMA: (82,264,299,460) -120 355
            #TAMANO MINIMO DE BOX:(31,31)
            #PROPORCION: ENTRE 0.71 Y 1.42 (cuadrada)
            for i in range(len(classes_c)) :
                if classes_c[i] == 1:
                    box_temp=boxes_c[i]
                    #print('box_temp:')
                    #print(box_temp)
                    if ((box_temp[2]-box_temp[0] >60 or box_temp[3]-box_temp[1] >100) 
                            and (((box_temp[2]+box_temp[0])/2)> 126 and 
                            ((box_temp[2]+box_temp[0])/2)<456 and
                            ((box_temp[3]+box_temp[1])/2)>0 and 
                            ((box_temp[3]+box_temp[1])/2)<640)):
                        boxes_c2.append(boxes_c[i])
                        scores_c2.append(scores_c[i])
                        classes_c2.append(classes_c[i])
                        print('se detecto un vehiculo cerca')
                if classes_c[i] == 2:
                    box_temp=boxes_c[i]
                    if ((box_temp[2]-box_temp[0] >42 or box_temp[3]-box_temp[1] >42) 
                            and (((box_temp[2]+box_temp[0])/2)> 46 and 
                            ((box_temp[2]+box_temp[0])/2)<215 and
                            ((box_temp[3]+box_temp[1])/2)>157 and 
                            ((box_temp[3]+box_temp[1])/2)<414)and 
                            ((box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])>0.71 and
                                (box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])<1.42)):
                        boxes_c2.append(boxes_c[i])
                        scores_c2.append(scores_c[i])
                        classes_c2.append(classes_c[i])
                        print('se detecto un semaforo rojo cerca')
                if classes_c[i] == 3:
                    box_temp=boxes_c[i]
                    if ((box_temp[2]-box_temp[0] >42 or box_temp[3]-box_temp[1] >42) 
                            and (((box_temp[2]+box_temp[0])/2)> 46 and 
                            ((box_temp[2]+box_temp[0])/2)<215 and
                            ((box_temp[3]+box_temp[1])/2)>157 and 
                            ((box_temp[3]+box_temp[1])/2)<414)and 
                            ((box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])>0.71 and
                                (box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])<1.42)):
                        boxes_c2.append(boxes_c[i])
                        scores_c2.append(scores_c[i])
                        classes_c2.append(classes_c[i])
                        print('se detecto un semaforo amarillo cerca')
                if classes_c[i] == 4:
                    box_temp=boxes_c[i]
                    if ((box_temp[2]-box_temp[0] >42 or box_temp[3]-box_temp[1] >42) 
                            and (((box_temp[2]+box_temp[0])/2)> 46 and 
                            ((box_temp[2]+box_temp[0])/2)<215 and
                            ((box_temp[3]+box_temp[1])/2)>157 and 
                            ((box_temp[3]+box_temp[1])/2)<414)and 
                            ((box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])>0.71 and
                                (box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])<1.42)):
                        boxes_c2.append(boxes_c[i])
                        scores_c2.append(scores_c[i])
                        classes_c2.append(classes_c[i])
                        print('se detecto un semaforo verde cerca')
                if classes_c[i] == 5:
                    box_temp=boxes_c[i]
                    if ((box_temp[2]-box_temp[0] >50 or box_temp[3]-box_temp[1] >50) 
                            and (((box_temp[2]+box_temp[0])/2)> 240 and 
                            ((box_temp[2]+box_temp[0])/2)<468 and
                            ((box_temp[3]+box_temp[1])/2)>11 and 
                            ((box_temp[3]+box_temp[1])/2)<636)and 
                            ((box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])>0.5 and
                                (box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])<2)):
                        boxes_c2.append(boxes_c[i])
                        scores_c2.append(scores_c[i])
                        classes_c2.append(classes_c[i])
                        print('se detecto un peaton cerca')
                if classes_c[i] == 6:
                    box_temp=boxes_c[i]
                    '''if ((box_temp[2]-box_temp[0] >40 or box_temp[3]-box_temp[1] >270) 
                            and (((box_temp[2]+box_temp[0])/2)> 244 and 
                            ((box_temp[2]+box_temp[0])/2)<407 and
                            ((box_temp[3]+box_temp[1])/2)>38 and 
                            ((box_temp[3]+box_temp[1])/2)<542)and 
                            ((box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])<0.3)):
                        boxes_c2.append(boxes_c[i])
                        scores_c2.append(scores_c[i])
                        classes_c2.append(classes_c[i])
                        print('se detecto una linea de pare cerca')'''
                    boxes_c2.append(boxes_c[i])
                    scores_c2.append(scores_c[i])
                    classes_c2.append(classes_c[i])
                    print('se detecto una linea de pare cerca')
                if classes_c[i] == 7:
                    box_temp=boxes_c[i]
                    #print(box_temp[2]-box_temp[0] )
                    #print(box_temp[3]-box_temp[1])
                    #print((box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1]))
                    if ((box_temp[2]-box_temp[0] >31 or box_temp[3]-box_temp[1] >31) 
                            and (((box_temp[2]+box_temp[0])/2)> 82 and 
                            ((box_temp[2]+box_temp[0])/2)<299 and
                            ((box_temp[3]+box_temp[1])/2)>264 and 
                            ((box_temp[3]+box_temp[1])/2)<460)and 
                            ((box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])>0.7 and
                                (box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])<1.5)):
                        boxes_c2.append(boxes_c[i])
                        scores_c2.append(scores_c[i])
                        classes_c2.append(classes_c[i])
                        print('se detecto un aviso de pare cerca')
                if classes_c[i] == 8:
                    box_temp=boxes_c[i]
                    if ((box_temp[2]-box_temp[0] >31 or box_temp[3]-box_temp[1] >31) 
                            and (((box_temp[2]+box_temp[0])/2)> 82 and 
                            ((box_temp[2]+box_temp[0])/2)<299 and
                            ((box_temp[3]+box_temp[1])/2)>264 and 
                            ((box_temp[3]+box_temp[1])/2)<460)and 
                            ((box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])>0.7 and
                                (box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])<1.5)):
                        boxes_c2.append(boxes_c[i])
                        scores_c2.append(scores_c[i])
                        classes_c2.append(classes_c[i])
                        print('se detecto un aviso de cruce en t cerca')
            
                if classes_c[i] == 9:
                    box_temp=boxes_c[i]
                    if ((box_temp[2]-box_temp[0] >31 or box_temp[3]-box_temp[1] >31) 
                            and (((box_temp[2]+box_temp[0])/2)> 82 and 
                            ((box_temp[2]+box_temp[0])/2)<299 and
                            ((box_temp[3]+box_temp[1])/2)>264 and 
                            ((box_temp[3]+box_temp[1])/2)<460)and 
                            ((box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])>0.71 and
                                (box_temp[2]-box_temp[0])/(box_temp[3]-box_temp[1])<1.42)):
                        boxes_c2.append(boxes_c[i])
                        scores_c2.append(scores_c[i])
                        classes_c2.append(classes_c[i])
                        print('se detecto un aviso de semaforo cerca')
            # print('boxesc2:')
            # print(boxes_c2)
            # print('scoresc2:')
            # print(scores_c2)
            # print('classesc2:')
            # print(classes_c2)
            scores_c2=np.array(scores_c2)
            boxes_c2=np.array(boxes_c2)
            classes_c2=np.array(classes_c2)
            box_clas=np.zeros((scores_c2.size,6), dtype="float32")
            for a in range(scores_c2.size):
                box_clas[a][0]=scores_c2[a].round(2)
                box_clas[a][1]=classes_c2[a]
                box_clas[a][2]=boxes_c2[a][0]
                box_clas[a][3]=boxes_c2[a][1]
                box_clas[a][4]=boxes_c2[a][2]
                box_clas[a][5]=boxes_c2[a][3]
            box_clas=np.float32(box_clas)
            #print('box_clas:')
            #print(box_clas.size)
            if box_clas.size is not 0:
                
                id_msg=identified_msg()
                #SE DEBE SERIALIZAR EL DATA
                id_msg.data=np.frombuffer(box_clas.tobytes(),'float32')
                id_msg.header= std_msgs.msg.Header()
                id_msg.header.seq=self.seq_obj
                id_msg.header.stamp=tm.now()
                id_msg.header.frame_id="Identified"
                self.objects_id.publish(id_msg)
                self.seq_obj+=1
                #rospy.loginfo("obj_det_node Message")
                #print(id_msg)
            else:
                print("no se detectaron objetos de interes")
            #print(id_msg)
            ind_intersection=False

        self.detect_obj==False
        #def mirar_msg(self, msg):
            #print('el subscriptor recibira:')
            #print('msg')s
            #print(msg)
            #print('time')
            #print(msg.header.stamp) 
    def activate_sl_cb(self,data):
        self.detect_stop = True
        self.deteccion_flag=True
        self.detect_obj=False
        print("OEDR on default configuration")    

    def visualizer_cb(self,data):
        self.visualizer=data.data

    def load_image_into_numpy_array(self, image):
        (im_width, im_height) = image.size
        return np.array(image.getdata()).reshape(
            (im_height, im_width, 3)).astype(np.uint8)

    def detect_objects(self, data):
        if data.header.stamp.secs - self.secImage >5 or self.secImage==0:
            self.secImage=data.header.stamp.secs
            if self.detect_obj==False and self.detect_stop == True:
                #print('Object detection disabled')
                #DETECCION DE TRIGGER
                print("tiempo que llega la imagen: secs "+str(data.header.stamp.secs)+" nsecs: "+str(data.header.stamp.nsecs))
                #------------------------------------------
                image_np =self.bridge.imgmsg_to_cv2(data)
                image_np_expanded = np.expand_dims(image_np, axis=0)
                image_np_org=image_np 
                image_np_org = cv2.cvtColor(image_np_org, cv2.COLOR_BGR2RGB)
                #PIL_image = PIL.Image.fromarray(np.uint8(image_np_org)).convert('RGB')
                #-----------color correct method 1------------
                #image_np_org2=to_pil(cca.automatic_color_equalization(from_pil(PIL_image)))
                #image_np_org = np.asarray(image_np_org2)
                self.image_full=image_np_org
                #---------------------------------------------
                #-----------color correct method 2------------
                #converter = PIL.ImageEnhance.Color(PIL_image)
                #img2 = converter.enhance(0.7)
                #img23= cv2.cvtColor(np.array(img2), cv2.COLOR_RGB2BGR)
                #gamma =1.4
                #gamma_c = np.array(255*(img23 / 255) ** gamma, dtype = 'uint8')
                #self.image_full=gamma_c
                #---------------------------------------------
                #plt.plot(image_np_org)
                #plt.imshow(image_np_org, interpolation='nearest')
                #plt.pause(1)
                #plt.close('all')
                #img= image_np_org[315:390, 75:450]
                #img= image_np_org[325:450, 100:540]
                img=self.image_full[325:450, 100:540]
                scale_percent = 50 # percent of original size
                width = int(img.shape[1] * scale_percent / 100)
                height = int(img.shape[0] * scale_percent / 100)
                dim = (width, height)
                # resize image
                resized = cv2.resize(img, dim, interpolation = cv2.INTER_AREA)

                #print('Resized Dimensions : ',resized.shape) 

                #Establecemos el rango minimo y maximo de (Blue, Green, Red):
                rojos_bajos = np.array([80,0,0])
                rojos_altos = np.array([255, 80, 80])
                # Recordatorio: el rango sirve para determinar que pixeles detectaremos.
                # Cada pixel tiene asignado un valor [B, G, R] segun su cantidad de Azul, Verde y Rojo.
                # Con el rango que hemos definido, detectaremos los pixeles que cumplan estas tres condiciones:
                # -Su valor B este entre 40 y 255
                # -Su valor G este entre 0 y 120
                # -Su valor R este entre 0 y 120


                #Detectamos los pixeles que esten dentro del rango que hemos establecido:
                mask = cv2.inRange(resized, rojos_bajos, rojos_altos)
                #mask = cv2.inRange(resized, rojos_bajos, rojos_altos)
                #print(mask.shape)

                if self.visualizer:
                    image_np_msg2=self.bridge.cv2_to_imgmsg(resized,"bgr8")
                    #self.stop_line_pub.publish(image_np_msg2)
                    image_np_msg3=self.bridge.cv2_to_imgmsg(mask,"mono8")
                    #self.mask_pub.publish(image_np_msg3)
                    plt.imshow(resized)
                    plt.pause(2)
                    plt.close('all')
                    plt.imshow(mask)
                    plt.pause(2)
                    plt.close('all')
                
                #print(np.asarray(mask))
                #im = Image.open(mask)
                black=0
                red=0
                for fila in np.asarray(mask):
                  for pixel in fila:
                      if pixel == 0:
                          black += 1
                      elif pixel ==255:
                          red += 1
                print("black = " + str(black) + "red = " + str(red))
                #time.sleep(1)
                #print("red"+str(red))
                if red >=2900:
                  #print('found stop line')
                  trigger=True
                else:
                  trigger=False
                #print("trigger"+str(trigger))
                #------------------------------------------
                #print(tm.now().secs-self.time_curr_trigger.secs)
                if trigger == False:
                    self.detect_obj==False
                    self.detect_stop == True
                    msg=std_msgs.msg.Bool()
                    msg.data=False
                    self.objects_sl.publish(msg)
                #ajustar este tiempo a 20 segundos para el sistema real, que son 12 de deteccion y 8 para
                # que avance en caso de que pueda y no se vea la linea de pare 
                if trigger == True and tm.now().secs-self.time_curr_trigger.secs  >20:
                    #print('entro al True')
                    self.detect_obj=True
                    self.detect_stop = False
                    msg=std_msgs.msg.Bool()
                    msg.data=True
                    self.objects_sl.publish(msg)
                    self.time_curr_trigger=tm.now()
                    if self.deteccion_flag== True:
                        self.deteccion(self.image_full)
                        self.deteccion_flag=False

#configurar la deteccion para que se active con bandera enviada de 




    def __init__(self, expandDis=0.8, goalSampleRate=0.5, maxIter=1000):
         
        #self.PATH_TO_CKPT = os.getcwd()+ '/src/object_detection_dt/scripts/modelo_congelado/frozen_inference_graph.pb'
        self.duckie='duckiebot2'
        self.visualizer=True
        self.detect_obj=False
        self.detect_stop=True
        self.deteccion_flag=True

        self.seq_obj=0
        self.seq_sce=0
        self.secImage=0

        self.time_curr_trigger=tm(0,0)

        rospack = rospkg.RosPack()
        #self.PATH_TO_CKPT = '/workspace/custom_ws/src/object_detection_dt/scripts/modelo_congelado/frozen_inference_graph.pb'
        self.PATH_TO_CKPT = rospack.get_path('object_detection_dt')+ '/scripts/modelo_congelado/frozen_inference_graph.pb'

        print('Loading model...')
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='') 
                print("se cargo el modelo")
        #time.sleep(10)
        self.bridge = CvBridge()
        self.new_img= rospy.Subscriber("%s/camera_node/image/raw" % (self.duckie),Image,self.detect_objects)
        self.visualizer_sub=rospy.Subscriber("%s/ads/visualizer" % (self.duckie),std_msgs.msg.Bool,self.visualizer_cb)
        self.activator_sl_sub=rospy.Subscriber("%s/object_detection/stop_line_detector" % (self.duckie),std_msgs.msg.Bool,self.activate_sl_cb)
        #self.new_msg= rospy.Subscriber("/Object_detection/identified",identified_msg ,self.mirar_msg)

        self.scene=rospy.Publisher("%s/path_planning/scene" % (self.duckie), scene_msg , queue_size=1)
        self.objects_id=rospy.Publisher("%s/object_detection/identified" % (self.duckie),identified_msg , queue_size=1)
        self.objects_sl=rospy.Publisher("%s/object_detection/stop_line" % (self.duckie),std_msgs.msg.Bool , queue_size=1)
        self.objects_img=rospy.Publisher("%s/object_detection/image" % (self.duckie), Image, queue_size=3)
        self.stop_line_pub=rospy.Publisher("%s/object_detection/stop_line_img" % (self.duckie), Image, queue_size=3)
        self.mask_pub=rospy.Publisher("%s/object_detection/stop_line_mask" % (self.duckie), Image, queue_size=1)

        #print("se inicializaron los publicadores y el subscriptor a el sensor")
        



def main(args):
    rospy.init_node('Object_detection', anonymous=True)
    rospy.loginfo("Start Object detection")
    #print("Start Object detection")
    obj_detec = obj_detection()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
    