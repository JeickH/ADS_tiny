
#!/usr/bin/python
# -*- coding: utf-8 -*-
u"""
@brief: Path Planning Sample Code with Randamized Rapidly-Exploring Random Trees (RRT)

@author: AtsushiSakai

@license: MIT

"""
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import random
import math
import copy
import obstacles


class RRT():
    u"""
    Class for RRT Planning
    """

    def __init__(self, start, goal, obstacleList,
                 randArea, mapa, expandDis=0.8, goalSampleRate=0.5, maxIter=1000):
        u"""
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Ramdom Samping Area [min,max]

        """
        self.start = Node(start[0], start[1])
        self.end = Node(goal[0], goal[1])
        self.minrandx = randArea[0]
        self.maxrandx = randArea[1]
        self.minrandy = randArea[2]
        self.maxrandy = randArea[3]
        self.expandDis = expandDis
        self.goalSampleRate = goalSampleRate
        self.maxIter = maxIter
        self.mapa = mapa

    def Planning(self, animation=True):
        u"""
        Pathplanning

        animation: flag for animation on or off
        """

        self.nodeList = [self.start]
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

        return path

    def DrawGraph(self, rnd=None):
        u"""
        Draw Graph
        """
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

        for (ox, oy, size) in obstacleList:
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


class Node():
    u"""
    RRT Node
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


if __name__ == '__main__':
    print("start RRT path planning")
    import matplotlib.pyplot as plt
    #import matplotrecorder
    #matplotrecorder.donothing = True

    # ====Search Path with RRT====
    #instruction can be 'right' or 'left' when mapa is 
    #'intersection' and 'traffic_light' or 'forward'
    #when mapa is 'traffic_light'  
    instruction= 'left'
    start=[8.75, 2.5]
    #obstacleList = obstacles.int_left
    #mapa can be 'intersection' or 'traffic_light'
    mapa='intersection'

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

    # Set Initial parameters
    rrt = RRT(start, goal,obstacleList,randArea,mapa)
    path = rrt.Planning(animation=True)
    print(str(path))
    # Draw final path
    rrt.DrawGraph()
    plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
    plt.grid(True)
    plt.pause(3)  # Need for Mac
    #  plt.show()
    #for i in range(10):
        #matplotrecorder.save_frame()  # save each frame

    #matplotrecorder.save_movie("animation.gif", 0.1)
