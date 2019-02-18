

#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
from sympy import *
from copy import deepcopy

#### YOUR IMPORTS GO HERE ####
import IPython
import Queue as Q
import numpy

#### END OF YOUR IMPORTS ####

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def euclideanDistance(config1,config2):
    distance = 0
    for x in range(3):
        distance += numpy.square(config1.point[x] - config2.point[x])
    return numpy.sqrt(distance)


def manhattan(config1,config2):
    return abs(config1.point[0]-config2.point[0]) + abs(config1.point[1]-config2.point[1]) + abs(config1.point[2]-config2.point[2])


def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

# black - [0,0,0], blue  - [0,0,1]
def printpnts(pnt,color): 
    handles.append(env.plot3(points = pnt,pointsize = .05,colors=color,drawstyle=1))



def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)


def ConvertPathToTrajectory(robot,path=[]):
#Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]

    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'') 
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj


class Node:
    def __init__(self,config):
        self.point = config
        self.parent = None
        self.H = 0
        self.G = 100
        self.e = 0



def collision_check(node,env,robot):
    Tz = matrixFromAxisAngle([0,0,node.point[2]])
    Target = array([[Tz[0][0],Tz[0][1],0,node.point[0]],
                [Tz[1][0],Tz[1][1],0,node.point[1]],
                [0,0,1,0.01],
                [0,0,0,1]])
    robot.SetTransform(Target)
    # robot.SetTransform(numpy.dot(Tz,robot.GetTransform()))
    return (env.CheckCollision(robot))



def neighbors_4c(currentNode,delta,env,robot):
    neighborNodelist = set()

    step = 0.25
    # delta4 = ((0,-step,0), #back
    #      (-step,0,0), 
    #      (0,step,0),
    #      (step,0,0))
    delta4 = ((0,-step,0), #back
         (-step,0,0), 
         (0,step,0),
         (step,0,0),
         (step,step,0),
         (-step,step,0),
         (step,-step,0),
         (-step,-step,0))
    if euclideanDistance(currentNode,goalconfig) < 2:
        # print 'taking step1'
        step = 0.05
        delta4 = ((0,-step,0), #back
         (-step,0,0), 
         (0,step,0),
         (step,0,0),
         (0,0,-1.5707963267948966),
         (0,0,1.5707963267948966))    

    for i in range(len(delta4)):
        neighborNode = Node(numpy.sum([currentNode.point,delta4[i]],axis=0))
        if neighborNode.point[2] < -1.58:
            neighborNode.point[2] = -1.5707963267948966
        elif neighborNode.point[2] > 1.58:
            neighborNode.point[2] = 1.5707963267948966
        neighborNode.H = euclideanDistance(neighborNode,goalconfig)
        if collision_check(neighborNode,env,robot): 
            printpnts((neighborNode.point[0],neighborNode.point[1],0),[1,0,0])
            # print i,('collision, neighbor discarded')
        else:
            printpnts((neighborNode.point[0],neighborNode.point[1],0),[0,0,1])
            neighborNodelist.add(deepcopy(neighborNode))
            # print i,('Added to neighborlist')
    return neighborNodelist


def cost_to_go(self,next):
    return euclideanDistance(self,next)


def check_ifgoal(currentNode,goalconfig):
    if euclideanDistance(currentNode,goalconfig) < 0.05:
        return True
    else:
        # print currentNode.point,goalconfig.point,'Not goal'
        return False 

E = 100
G = 100

def key_e(node,G):
    node.e = (G -node.G)/node.H
    return node.e

def contains(list,node):
    for x in list:
        if x.point[0] == node.point[0] and x.point[1] == node.point[1] and x.point[2] == node.point[2]:
            return True
    return False

def ImproveSolution():
    while len(openlist)>0:
        currentNode = max(openlist,key=lambda o:o.e)
        print 'current node - ',currentNode.point, currentNode.e
        openlist.remove(currentNode)
        closedlist.add(deepcopy(currentNode))
        global E
        global G
        if currentNode.e < E:
            E = currentNode.e
        if check_ifgoal(currentNode,goalconfig):
            G = currentNode.G
            return G

        for neighborNode in neighbors_4c(currentNode,delta4,env,robot):
            if contains(closedlist,neighborNode):
                continue
            if (currentNode.G + euclideanDistance(neighborNode,currentNode)) < neighborNode.G:
                neighborNode.G = currentNode.G + euclideanDistance(neighborNode,currentNode)
                neighborNode.parent = currentNode
                if (neighborNode.G + neighborNode.H) < G:
                    neighborNode.e = key_e(neighborNode,G) 
                    openlist.add(deepcopy(neighborNode))


if __name__ == "__main__":

    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)


    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('data/pr2test2.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to thvariable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);

    

    with env:
        # the active DOF are translation in X and Y and rotation about the Z axis of the base of the robot.
        robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

        goalpoint = [2.6,-1.3,-pi/2]
        start = time.clock()
        #### YOUR CODE HERE ####
            
    
        startpoint = (numpy.append(robot.ComputeAABB().pos()[0:2],0))

        #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
    
        openlist = set()
        closedlist = set()


        spinner = None
        handles = []   

        
        
        counter = 0

        startconfig = Node(startpoint)
        goalconfig = Node(goalpoint)
        # goalconfig = Node([-0.36137935,-1.4,45]) # array([-3.36137935, -1.4 ,0])
        startconfig.G = 0
        startconfig.H = euclideanDistance(startconfig,goalconfig)

        currentNode = startconfig
        currentNode.e = (G -currentNode.G)/currentNode.H

        openlist.add(deepcopy(currentNode))

        step = 0.25
        delta4 = ((0,-step,0), #back
             (-step,0,0), 
             (0,step,0),
             (step,0,0),
             (step,step,0),
             (-step,step,0),
             (step,-step,0),
             (-step,-step,0))

        while len(openlist)>0:
            
            ImproveSolution()
            print 'G - ', G
            counter = counter +1
            print 'counter-', counter
            if counter >50:
                break

            for i in openlist:
                i.e = key_e(i,G)
                if (i.G + i.H) >= G:
                    openlist.remove(i)




        # printpnts(path,[0,0,1])

        #### Draw the X and Y components of the configurations explored by your algorithm

        # IPython.embed()

    path = [] #put your final path in this variable
        #### END OF YOUR CODE ###
    end = time.clock()
    print "Time: ", end - start

        # Now that you have computed a path, convert it to an openrave trajectory 
    traj = ConvertPathToTrajectory(robot, path)

    # Execute the trajectory on the robot.
    if traj != None:
        robot.GetController().SetPath(traj)


waitrobot(robot)

raw_input("Press enter to exit...")




# hw2_astar.py
# Displaying hw2_astar.py.