

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
    for x in range(2):
        distance += numpy.square(config1.point[x] - config2.point[x])
    return numpy.sqrt(distance)


def manhattan(config1,config2):
    distance = 0
    for x in range(2):
        distance += numpy.sum(numpy.subtract(config1.point,config2.point))
    return distance


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
        self.G = 0



# class config:
#     def __init__(self,array3d):
#         self.x = Float(str(array3d[0]),2)
#         self.y = Float(str(array3d[1]),2)
#         self.th = Float(str(array3d[2]),2)
#     def __repr__(self):
#         return "(%s,%s,%s)"%(self.x,self.y,self.th)



def collision_check(node,env,robot):
    Target = array([[1,0,0,node.point[0]],
                [0,1,0,node.point[1]],
                [0,0,1,0.01],
                [0,0,0,1]])
    # Tz = matrixFromAxisAngle([0,0,numpy.node.point[2]])
    robot.SetTransform(Target)
    # robot.SetTransform(numpy.dot(Tz,robot.GetTransform()))
    return (env.CheckCollision(robot))



def neighbors_4c(currentNode,delta,env,robot):
    neighborNodelist = set()
    for i in range(len(delta4)):
        neighborNode = Node(numpy.sum([currentNode.point,delta4[i]],axis=0))
        neighborNode.H = euclideanDistance(neighborNode,goalconfig)
        if collision_check(neighborNode,env,robot): 
            printpnts((neighborNode.point),[0,0,0])
            print i,('collision, neighbor discarded')
        else:
            printpnts(neighborNode.point,[0,0,1])
            neighborNodelist.add(deepcopy(neighborNode))
            print i,('Added to neighborlist')
    return neighborNodelist


def cost_to_go(self,next):
    return euclideanDistance(self,next)


def check_ifgoal(currentNode,goalconfig):
    if euclideanDistance(currentNode,goalconfig) < 0.01:
        return True
    else:
        print currentNode.point,goalconfig.point,'Not goal'
        return False 


def contains(list,node):
    for x in list:
        if x.point[0] == node.point[0] and x.point[1] == node.point[1] and x.point[2] == node.point[2]:
            return True
    return False




# def A_Star(start,goal,env,robot,hand)


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

        goalconfig = [2.6,-1.3,-pi/2]
        start = time.clock()
        #### YOUR CODE HERE ####
            
    
        startpoint = (numpy.append(robot.ComputeAABB().pos()[0:2],0))

        #### Implement your algorithm to compute a path for the robot's base starting from the current configuration of the robot and ending at goalconfig. The robot's base DOF have already been set as active. It may be easier to implement this as a function in a separate file and call it here.
    
        openlist = set()
        closedlist = set()


        spinner = None
        handles = []   

        startconfig = Node(startpoint)
        goalconfig = Node(goalconfig)
        # goalconfig = Node([-0.36137935,-1.4,45]) # array([-3.36137935, -1.4 ,0])
        startconfig.G = 0
        startconfig.H = euclideanDistance(startconfig,goalconfig)

        currentNode = startconfig
        openlist.add(deepcopy(currentNode))
        counter = 0
 
        while len(openlist) > 0:
            if euclideanDistance(currentNode,goalconfig) < 0.1:
                step = 0.07
            elif euclideanDistance(currentNode,goalconfig) < 0.5:
                step = 0.01
            else:
                step = 0.25


            delta4 = ((0,-step,0), #back
                 (-step,0,0), 
                 (0,step,0),
                 (step,0,0))
            

            counter = counter +1
            if counter > 200:
                break
            print 'counter=',counter
            currentNode = min(openlist,key=lambda o:o.H+o.G)
            print 'currentNode - ', currentNode.point

            # #Listing Openlist
            # for node in openlist:
            #     print node.point,(node.H + node.G)

            if check_ifgoal(currentNode,goalconfig):
                print "DONE!"
                path = []
                while currentNode.parent==startconfig:
                    path.append(deepcopy(currentNode.point))
                    currentNode = currentNode.parent
                path.append(currentNode)
                path.reverse()
                # return path

            else:
                openlist.remove(currentNode)
                closedlist.add(deepcopy(currentNode))
                for neighborNode in neighbors_4c(currentNode,delta4,env,robot):
                    # print 'neighbor-', neighborNode.point

                    #Listing closedlist
                    # for node in closedlist:
                    #     print 'close list - ',node.point,(node.H + node.G)


                    if contains(closedlist,neighborNode):
                        continue
                    temp_G = currentNode.G + euclideanDistance(currentNode,neighborNode)
                    openlist.add(deepcopy(neighborNode))
                    # print neighborNode.point 
                    neighborNode.G = temp_G
                    neighborNode.parent = currentNode
                    # print 'size of open list-',len(openlist)





                    # if len(openlist) ==0:
                    #     print ('Initializing first step')
                    #     neighborNode.G = temp_G
                    #     neighborNode.parent = currentNode
                        
                    # else:
                    #     # if not contains(openlist,neighborNode):
                    #         print 'Second step'
                    #         # neighborNode.H = euclideanDistance(neighborNode,goalconfig)
                    #         neighborNode.parent = currentNode
                    #         neighborNode.G = temp_G
                    #         # openlist.add(deepcopy(neighborNode))




        # path = [[-2,-1.4,0],[-1.5,-1.4,0.1],[-1,-1.4,45],[-0.5,-1.4,45]]
        # currnetNode = Node([-0.5,-1.4,45])
        # a = neighbors_4c(startconfig,delta4,env,robot)




        


        # printpnts(path,[0,0,1])






        #### Draw the X and Y components of the configurations explored by your algorithm

    IPython.embed()

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