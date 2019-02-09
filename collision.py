#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
import IPython


if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def tuckarms(env,robot):
    with env:
        jointnames = ['l_shoulder_lift_joint','l_elbow_flex_joint','l_wrist_flex_joint','r_shoulder_lift_joint','r_elbow_flex_joint','r_wrist_flex_joint']
        robot.SetActiveDOFs([robot.GetJoint(name).GetDOFIndex() for name in jointnames])
        robot.SetActiveDOFValues([1.29023451,-2.32099996,-0.69800004,1.27843491,-2.32100002,-0.69799996]);
        robot.GetController().SetDesired(robot.GetDOFValues());
    waitrobot(robot)

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
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # tuck in the PR2's arms for driving
    tuckarms(env,robot);


    #### YOUR CODE HERE ####
    Target = array([[1,0,0,-.15],
                [0,1,0,0],
                [0,0,1,.0499],
                [0,0,0,1]])

    robot.SetTransform(Target)  

    col = env.CheckCollision(robot)
    print col


    # env.drawlinestrip(points=array(((-1,-0.5,0),(-1,0.5,0),(-1.5,1,0))),linewidth=3.0)
    with env:
        robot.SetActiveDOFs([27,28,29,30,31,32,33])
        robot.SetActiveDOFValues([0,0,0,0,0,0,0])
        col = env.CheckCollision(robot)
        print col
        # IPython.embed()
        values = robot.GetDOFValues()
        robot.GetController().SetDesired(values)
        # time.sleep(1.5)
    waitrobot(robot)

    # 

    #### END OF YOUR CODE ###


    raw_input("Press enter to exit...")
    env.Destroy()