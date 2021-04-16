#!/usr/bin/env python2
#brief: Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)
#Author: AtsushiSakai
#license: MIT

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import std_msgs.msg
import random
import math
import copy
import obstacles
import roslib
import sys
import rospy


class RRT():
    
    #Class for RRT Planning
   
    def Planning(self, data, animation=True):
        
        #Pathplanning
        """while not self.TIinstruction:
            print("No se ha recibido ninguna instruccion")
            rospy.spin()"""
                    #animation: flag for animation on or off
        print("se recibio una nueva senal de escena")
        data=str(data)
        data=data.replace('data: ','')
        data=data.replace('\"','')
        self.mapa=data
        print("se modifico el mapa como "+str(data))

        if self.instruction == 'right' and self.mapa == 'intersection':
            goal=[11.25, 6.25]
            randArea=[7.5, 12.0, 2.5, 7.5]
            obstacleList = obstacles.int_right

        if self.instruction == 'left' and self.mapa == 'intersection':
            goal=[3.75, 8.75]
            randArea=[2.5, 10, 2.5, 10]
            obstacleList = obstacles.int_left

        if self.instruction == 'right' and self.mapa == 'traffic_light':
            goal=[11.25, 6.25]
            randArea=[7.5, 12.0, 2.5, 7.5]
            obstacleList = obstacles.tl_right

        if self.instruction == 'left' and self.mapa == 'traffic_light':
            goal=[3.75, 8.75]
            randArea=[2.5, 10, 2.5, 10]
            obstacleList = obstacles.tl_left

        if self.instruction == 'forward' and self.mapa == 'traffic_light':
            goal=[8.75, 12.5]
            randArea=[8, 9.5, 2.5, 15]
            obstacleList = obstacles.tl_for
        
        #print("current instruction: ")
        #print(self.instruction)
        #print("current map: ")
        #print(self.mapa)
        #print("Search Area:")
        #print(str(randArea))
        self.end = Node(goal[0], goal[1])
        self.minrandx = randArea[0]
        self.maxrandx = randArea[1]
        self.minrandy = randArea[2]
        self.maxrandy = randArea[3]
        self.obstacleList=obstacleList


        self.nodeList = [self.start]

        print("se definio una configuracion")
        while True:
            rnd = [random.uniform(self.minrandx, self.maxrandx), random.uniform(
                    self.minrandy, self.maxrandy)]
            # Random Sampling
            #if random.randint(0, 100) > self.goalSampleRate:
                #rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                    #self.minrand, self.maxrand)]
            #else:
                #rnd = [self.end.x, self.end.y]
            print(rnd)
            # Find nearest node
            nind = self.GetNearestListIndex(self.nodeList, rnd)
            print(nind)

            # expand tree
            nearestNode = self.nodeList[nind]
            theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

            newNode = copy.deepcopy(nearestNode)
            newNode.x += self.expandDis * math.cos(theta)
            newNode.y += self.expandDis * math.sin(theta)
            newNode.parent = nind

            if self.__CollisionCheck(newNode, obstacleList)==True:
                self.nodeList.append(newNode)
            #continue


            #self.nodeList.append(newNode)

            # check goal
            dx = newNode.x - self.end.x
            dy = newNode.y - self.end.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= self.expandDis:
                #plt.pause(0.5)
                llego=True
                print("Goal!!")
                break
            #cont1=0
            if animation:
                #if cont1 == 0:
                #    fig = plt.figure()
                #self.DrawGraph(rnd,fig)
                self.DrawGraph(rnd)
                #cont1+=1

        path = [[self.end.x, self.end.y]]
        lastIndex = len(self.nodeList) - 1
        while self.nodeList[lastIndex].parent is not None:
            node = self.nodeList[lastIndex]
            path.append([round(node.x,2), round(node.y,2)])
            lastIndex = node.parent
        path.append([self.start.x, self.start.y])
# Draw final path
        if llego:
            llego=False
            self.DrawGraph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(5)  # Need for Mac
                    #  plt.show()"""
        #PUBLICAR EL PATH 
        self.new_path.publish(str(path))
        self.TIinstruction=False
        print("se publico un nuevo path")

    def DrawGraph(self, rnd=None):
        #Draw Graph
        plt.clf()
        #DIBUJO DE MAPA
        rect1=patches.Rectangle((0,0),5,5,linewidth=1,
                edgecolor='g',facecolor='green')
        rect2=patches.Rectangle((10,0),5,5,linewidth=1,
                edgecolor='g',facecolor='green')
        rect3=patches.Rectangle((5,0),5,5,linewidth=1,
                edgecolor='k',facecolor='black')
        rect4=patches.Rectangle((0,5),15,5,linewidth=1,
                edgecolor='k',facecolor='black')
        rect5=patches.Rectangle((7.3,0.5),0.4,1.5,linewidth=1,
                edgecolor='k',facecolor='yellow')
        rect6=patches.Rectangle((7.3,3.5),0.4,1.5,linewidth=1,
                edgecolor='k',facecolor='yellow')
        rect7=patches.Rectangle((7.7,4.4),2.3,0.6,linewidth=1,
                edgecolor='r',facecolor='red')
        rect8=patches.Rectangle((3.5,7.3),1.5,0.4,linewidth=1,
                edgecolor='k',facecolor='yellow')
        rect9=patches.Rectangle((0.5,7.3),1.5,0.4,linewidth=1,
                edgecolor='k',facecolor='yellow')
        rect10=patches.Rectangle((10,7.3),1.5,0.4,linewidth=1,
                edgecolor='k',facecolor='yellow')
        rect11=patches.Rectangle((13,7.3),1.5,0.4,linewidth=1,
                edgecolor='k',facecolor='yellow')
        
        plt.gca().add_patch(rect1)
        plt.gca().add_patch(rect2)
        plt.gca().add_patch(rect3)
        plt.gca().add_patch(rect4)
        plt.gca().add_patch(rect5)
        plt.gca().add_patch(rect6)
        plt.gca().add_patch(rect7)
        plt.gca().add_patch(rect8)
        plt.gca().add_patch(rect9)
        plt.gca().add_patch(rect10)
        plt.gca().add_patch(rect11)
        
        if self.mapa == 'intersection':
            rect12=patches.Rectangle((0,10),15,5,linewidth=1,
                edgecolor='g',facecolor='green')
            rect13=patches.Rectangle((10,10),1,1,linewidth=2,
                angle=45,edgecolor='k',facecolor='yellow')
            rect14=patches.Rectangle((9.6,10.65),0.8,0.2,linewidth=1,
                edgecolor='k',facecolor='black')
            rect15=patches.Rectangle((9.9,10.35),0.2,0.5,linewidth=1,
                edgecolor='k',facecolor='black')
            plt.gca().add_patch(rect12)
            plt.gca().add_patch(rect13)
            plt.gca().add_patch(rect14)
            plt.gca().add_patch(rect15)

        if self.mapa == 'traffic_light':
            rect12=patches.Rectangle((0,10),5,5,linewidth=1,
                edgecolor='g',facecolor='green')
            rect13=patches.Rectangle((10,10),5,5,linewidth=1,
                edgecolor='g',facecolor='green')
            rect14=patches.Rectangle((5,10),5,5,linewidth=1,
                edgecolor='k',facecolor='black')
            rect15=patches.Rectangle((7.3,10),0.4,1.5,linewidth=1,
                edgecolor='k',facecolor='yellow')
            rect16=patches.Rectangle((7.3,13),0.4,1.5,linewidth=1,
                edgecolor='k',facecolor='yellow')
            rect17=patches.Rectangle((10.8,10),1,1,linewidth=2,
                angle=45,edgecolor='k',facecolor='yellow')
            rect18=patches.Rectangle((10.7,10.3),0.2,0.8,linewidth=2,
                edgecolor='k',facecolor='black')
            circle1=patches.Circle((10.8,10.95),radius=0.08,
                edgecolor='r',facecolor='red')
            circle2=patches.Circle((10.8,10.7),radius=0.08,
                edgecolor='y',facecolor='yellow')
            circle3=patches.Circle((10.8,10.45),radius=0.08,
                edgecolor='g',facecolor='green')
            plt.gca().add_patch(rect12)
            plt.gca().add_patch(rect13)
            plt.gca().add_patch(rect14)
            plt.gca().add_patch(rect15)
            plt.gca().add_patch(rect16)
            plt.gca().add_patch(rect17)
            plt.gca().add_patch(rect18)
            plt.gca().add_patch(circle1)
            plt.gca().add_patch(circle2)
            plt.gca().add_patch(circle3)
        
        
        #-------------
        if rnd is not None:
            plt.plot(rnd[0], rnd[1], "^k")
        for node in self.nodeList:
            if node.parent is not None:
                plt.plot([node.x, self.nodeList[node.parent].x], [
                         node.y, self.nodeList[node.parent].y], "-b")

        for (ox, oy, size) in self.obstacleList:
            plt.plot(ox, oy, "om", ms=10 * size)

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis([0, 15, 0, 15])
        plt.grid(True)
        plt.pause(0.01)
        #matplotrecorder.save_frame()  # save each frame

    def GetNearestListIndex(self, nodeList, rnd):
        dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
                 ** 2 for node in nodeList]
        minind = dlist.index(min(dlist))
        return minind

    def __CollisionCheck(self, node, obstacleList):

        for (ox, oy, size) in obstacleList:
            dx = ox - node.x
            dy = oy - node.y
            d = math.sqrt(dx * dx + dy * dy)
            if d <= size:
                return False  # collision

        return True  # safe

    def new_instruction_callback(self, data):
        print("se recibio una nueva instruccion")
        data=str(data)
        data=data.replace('data: ','')
        data=data.replace('\"','')
        self.instruction=data
        self.TIinstruction=True
        print("se modifico instruction como "+ str(data))
    
    def __init__(self, expandDis=0.8, goalSampleRate=0.5, maxIter=1000):
    #instruction can be 'right' or 'left' when mapa is 
    #'intersection' and 'traffic_light' or 'forward'
    #when mapa is 'traffic_light'  
    #obstacleList = obstacles.int_left
    #mapa can be 'intersection' or 'traffic_light'
        #self.instruction=instruction
        #subscribirse al topico scene y al recibir la senal de un mapa
        #realizar el planning, este retorna el path
        self.car_path=rospy.Subscriber("/Path_planning/scene",std_msgs.msg.String,self.Planning,queue_size=1)
        self.new_path=rospy.Publisher("/Path_planning/path", std_msgs.msg.String, queue_size=1)
        self.new_instruction= rospy.Subscriber("/Path_planning/instruction",std_msgs.msg.String,self.new_instruction_callback)
        print("se inicializo el subscriptor car_path y publicador new_path")
        start=[8.75, 2.5]
        self.start = Node(start[0], start[1])
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.TIinstruction=False
        #self.mapa = mapa



class Node():
    
    #RRT Node
    

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


def main(args):
    rospy.init_node('Path_planning', anonymous=True)
    rospy.loginfo("Start RRT path planning")
    print("Start RRT path planning")
    rrt = RRT()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
    