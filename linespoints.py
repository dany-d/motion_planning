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


    spinner = None
    handles = []
    
    handles.append(env.drawlinestrip(points=array(((3.95,1.95,0.74),(3.1,1.95,0.74),(3.1,0.45,0.74),(3.95,0.45,0.74),(3.95,1.95,0.74))),linewidth=3.0))
    handles.append(env.drawlinestrip(points=array(((3.1,-1.95,0.74),(3.95,-1.95,0.74),(3.95,-0.45,0.74),(3.1,-0.5,0.74),(3.1,-1.95,0.74))),linewidth=3.0))
    handles.append(env.drawlinestrip(points=array(((-3.1,-0.8,0.74),(-0.38,-0.8,0.74),(-0.38,0.8,0.74),(-3.1,0.8,0.74),(-3.1,-0.8,0.74))),linewidth=5.0))





    angle = linspace(0,360,35)
    point = zeros((35,3))
    point[:,0] = 6*sin(angle)
    point[:,1] = 6*cos(angle)
    point[:,2] = 0.9
    handles.append(env.plot3(points = point,pointsize = .05,colors=array(((0,0,1))),drawstyle=1))

    time.sleep(10)



    raw_input("Press enter to exit...")
    env.Destroy()

