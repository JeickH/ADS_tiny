#!/usr/bin/env python


from __future__ import print_function
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
from action_node.srv import path_planning, path_planningResponse
import numpy as np

def DrawGraph(mapa,nodeList,obstacleList , start, end,carList, rnd=None):
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
    
    if mapa == 'intersection':
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

    if mapa == 'traffic_light':
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
    for node in nodeList:
        if node.parent is not None:
            plt.plot([node.x, nodeList[node.parent].x], [
                     node.y, nodeList[node.parent].y], "-b")

    #for (ox, oy, size) in obstacleList:
        #plt.plot(ox, oy, "om", ms=10 * size)
    #Des-serializar el carlist en carlist2
    N = 3
    carList2 = [carList[n:n+N] for n in range(0, len(carList), N)]

    for (ox, oy, size) in carList2:
        plt.plot(ox, oy, "om", ms=30 * size)

    plt.plot(start.x, start.y, "xr")
    plt.plot(end.x, end.y, "xr")
    plt.axis([0, 15, 0, 15])
    plt.grid(True)
    plt.pause(0.01)
    #matplotrecorder.save_frame()  # save each frame

def GetNearestListIndex(nodeList, rnd):
    dlist = [(node.x - rnd[0]) ** 2 + (node.y - rnd[1])
             ** 2 for node in nodeList]
    minind = dlist.index(min(dlist))
    return minind

def __CollisionCheck(node, obstacleList):

    for (ox, oy, size) in obstacleList:
        dx = ox - node.x
        dy = oy - node.y
        d = math.sqrt(dx * dx + dy * dy)
        if d <= size:
            return False  # collision

    return True  # safe

class Node():
    
    #RRT Node
    

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None

def make_path_planning(req):
    instruction=req.instruction
    mapa=req.scene
    carList=req.objects
    rospy.loginfo("Path Planning Server Message")
    print("se modifico el mapa como "+mapa)

    if instruction == 'forward' and mapa == 'intersection':
        instruction='right'
    if instruction == 'left' and mapa == 'traffic_light':
        instruction='right'

    if instruction == 'right' and mapa == 'intersection':
        goal=[11.25, 6.25]
        randArea=[7.5, 12.0, 2.5, 7.5]
        obstacleList = obstacles.int_right

    if instruction == 'left' and mapa == 'intersection':
        goal=[3.75, 8.75]
        randArea=[2.5, 10, 2.5, 10]
        obstacleList = obstacles.int_left

    if instruction == 'right' and mapa == 'traffic_light':
        goal=[11.25, 6.25]
        randArea=[7.5, 12.0, 2.5, 7.5]
        obstacleList = obstacles.tl_right

    if instruction == 'left' and mapa == 'traffic_light':
        goal=[3.75, 8.75]
        randArea=[2.5, 10, 2.5, 10]
        obstacleList = obstacles.tl_left

    if instruction == 'forward' and mapa == 'traffic_light':
        goal=[8.75, 12.5]
        randArea=[8, 9.5, 2.5, 15]
        obstacleList = obstacles.tl_for
    #print("current instruction: ")
    #print(self.instruction)
    #print("current map: ")
    #print(self.mapa)
    #print("Search Area:")
    #print(str(randArea))
    end = Node(goal[0], goal[1])
    minrandx = randArea[0]
    maxrandx = randArea[1]
    minrandy = randArea[2]
    maxrandy = randArea[3]
    obstacleList=obstacleList
    start=Node(8.75, 2.5)
    expandDis=1.9
    goalSampleRate=0.2
    maxIter=1000
    animation=req.animation
    nodeList = [start]
    fin= True
    #print("se definio una configuracion")
    while fin:
        rnd = [random.uniform(minrandx, maxrandx), random.uniform(
                minrandy, maxrandy)]
        # Random Sampling
        #if random.randint(0, 100) > self.goalSampleRate:
            #rnd = [random.uniform(self.minrand, self.maxrand), random.uniform(
                #self.minrand, self.maxrand)]
        #else:
            #rnd = [self.end.x, self.end.y]
        #print(rnd)
        # Find nearest node
        nind = GetNearestListIndex(nodeList, rnd)
        #print(nind)

        # expand tree
        nearestNode = nodeList[nind]
        theta = math.atan2(rnd[1] - nearestNode.y, rnd[0] - nearestNode.x)

        newNode = copy.deepcopy(nearestNode)
        newNode.x += expandDis * math.cos(theta)
        newNode.y += expandDis * math.sin(theta)
        newNode.parent = nind

        if __CollisionCheck(newNode, obstacleList)==True:
            nodeList.append(newNode)
        #continue


        #self.nodeList.append(newNode)

        # check goal
        dx = newNode.x - end.x
        dy = newNode.y - end.y
        d = math.sqrt(dx * dx + dy * dy)
        if d <= expandDis:
            #plt.pause(0.5)
            llego=True
            #print("Goal!!")
            fin=False
            break
        #cont1=0
        if animation:
            #if cont1 == 0:
            #    fig = plt.figure()
            #self.DrawGraph(rnd,fig)
            DrawGraph(mapa, nodeList, obstacleList,start,end,carList, rnd)
            #cont1+=1


    path = [[end.x, end.y]]
    lastIndex = len(nodeList) - 1
    while nodeList[lastIndex].parent is not None:
        node = nodeList[lastIndex]
        path.append([round(node.x,2), round(node.y,2)])
        lastIndex = node.parent
    path.append([start.x, start.y])
# Draw final path
    if llego:
        llego=False
        DrawGraph(mapa,nodeList, obstacleList,start,end, carList)
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.grid(True)
        plt.pause(3) 
        plt.close() # Need for Mac
                #  plt.show()"""
    #PUBLICAR EL PATH 
    #print('path:')
    #print(path)
    #print('path.size')
    #print(len(path))
    path2=np.zeros((len(path),2),dtype="float32")
    i=0
    for ni in path:
        path2[i][0]=round(ni[0],2)
        path2[i][1]=round(ni[1],2)
        i+=1
    #print('path2:')
    rospy.loginfo("Path found!")
    path3=path2[::-1]
    #print(path3)
    #fin=False
    return path_planningResponse(np.frombuffer(path3.tobytes(),'float32'))
    

def path_planning_server():
    rospy.init_node('path_planning_server')
    s = rospy.Service('pathPlanning', path_planning, make_path_planning)
    rospy.loginfo("Start Path Planning Server")
    s.spin()

if __name__ == "__main__":
    path_planning_server()