#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""

Pure Pursuit path tracking algorithm:

Given current pose
    p_curr=traj(t)
and target pose
    p_goal=traj(t+dt),
the selection of (v, w) follows this idea:
    v: reduce the distance between p_curr and p_goal.
    w: orient the turtle towards p_goal.

The proportional control is adopted to reduce the error.

Problem:
When there is a curved traj, the robot can't accurately track the traj.
Instread, the robot's traj will be inside the real trajectory (a smaller turn).

"""

import numpy as np
import matplotlib.pyplot as plt
import math
import copy
import time

import os,sys
FILE_PATH = os.path.join(os.path.dirname(__file__))
IMAGE_PATH=FILE_PATH+'/../images'
sys.path.append(FILE_PATH + "/../lib_mylib")

from mylib import calc_dist, angle_pi_to_pi, bound
from myrobot import PIDcontroller, RobotModel_SingleTrack
from mytrajectory import Trajectory, Path
from myspline import Spline
from mycv import mycv

ANIMATION_ON = False
ANIMATION_GAP = 50
    
def plot_traj(traj):
    (x_list, y_list, yaw_list) = traj.query_pose()
    plt.plot(x_list, y_list,'b-o', ms=2)
    plt.plot(xl0, yl0, 'ro',ms=4)

def main_PurePursuit(robot, traj, T):

    dt = 0.1  # control period
    traj.compute_discrete_trajecotry(t_start=0, t_end=T, dt=dt)
    v_mean=traj.total_length/T
    idea_dist=0.1
    goal_dist=0.2

    # start
    t_curr = robot.query_time()
    path=Path() # record path
    path.append(robot.x, robot.y, robot.yaw)

    # for step in range(1, int(T/dt) + 100 ):
    for step in range(1, 1000 ):

        # set the goal as the point after 2*dt seconds
        t_goal = min(t_curr + idea_dist/v_mean, T)

        # get current pose and goal pose
        (x_curr, y_curr, yaw_curr) = (robot.x, robot.y, robot.yaw)
        (x_next, y_next, yaw_next) =  traj.query_pose(t=t_goal)

        # angle error -- reduce it
        angle_target = np.arctan2(y_next-y_curr, x_next-x_curr)
        yaw_error = angle_pi_to_pi(
            angle_target-yaw_curr)  # to [-pi, pi]

        # distance error to the goal point -- reduce it
        dist_error = calc_dist(x_curr, y_curr, x_next, y_next)

        # control (v, w)
        if step == 1:
            pid_v = PIDcontroller(P=2) # If dis=0.1m, we want v=0.1m/s
            pid_w = PIDcontroller(P=4, D=0)
        v = pid_v.feedback(dist_error-idea_dist)
        w = pid_w.feedback(yaw_error)

        # output
        robot.move(v, w, dt)
        path.append(robot.x, robot.y, robot.yaw)

        # print current state
        t_curr = robot.query_time()
        print("step=%d, t=%.3f, x=%.2f, y=%.2f, v=%.2f, w=%.2f, yaw=%.2f, theta=%.2f; d_dist=%.2f, d_yaw=%.2f"
                % (step, t_curr, robot.x, robot.y, v, w,
                robot.yaw/np.pi*180, robot.theta/np.pi*180,
                dist_error, yaw_error))

        # check if reach target
        reach_target = False
        if calc_dist(x_curr, y_curr, traj.x_list[-1], traj.y_list[-1])<goal_dist:
            reach_target=True

        # plot
        if (ANIMATION_ON and (step % ANIMATION_GAP == 0)) or reach_target:
            ar=[float(x) for x in [x_curr, y_curr, 0.2*math.cos(robot.yaw), 0.2*math.sin(robot.yaw)]]
            plt.arrow(ar[0],ar[1],ar[2],ar[3],
                head_width=0.1, head_length=0.1, linewidth=5, fc='k', ec='k')
            plot_traj(traj)
            plt.plot(path.xl, path.yl, 'r-,')
            plt.plot(x_next, y_next, 'xr',ms=12) # next
            plt.plot(x_curr, y_curr, 'ok',ms=12)  # current 

            plt.pause(0.0001)
            if reach_target:
                mycv.savefig(IMAGE_PATH+"/PurePursuit.png")
            plt.show()
        
        if reach_target:
            print "\nReach the target !!!"
            path.plot_velocity(dt, x_axis_time=robot.query_time())
            mycv.savefig(IMAGE_PATH+"/PurePursuit_vel.png")
            plt.show()
            break

    return # end of PurePursuit

if __name__ == '__main__':

    # Configurations:

    # It's a 2m*2m ground, and a 0.2m*0.2m robot moving at 0.5m/s. 
    # The whole traj length is about 10m,
    # so it will take 30s to drive throught it

    # set path points
    xl0 = [0, 1, 5, 9, 11, 10, 6, 5, 2]
    yl0 = [0, 3, 5, -1, -4, 5, -3, 1, 1]
    xl0=np.array(xl0)/5.0
    yl0=np.array(yl0)/5.0
    T=30 # total time (s)
    
    # generate path funcs, and a traj instance
    spline=Spline(xl0, yl0, T)
    traj=Trajectory(spline.fxt, spline.fyt, param=None)

    # set robot model
    robot=RobotModel_SingleTrack()
    main_PurePursuit(robot, traj, T)