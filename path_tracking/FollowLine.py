#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""

"Follow a Line" path tracking algorithm:

Given a trajectory (x(t),y(t)), 
we want to reduce the distance between robot and this traj
by setting a proper steering angle, and meanwhile keep a 
constant velocity moving forward.

In conclusion, the selection of (v, w) follows this idea:
    v: stays the same as distance / time
    w:  1. drive robot closer to traj. 
        2. make robot alligned with the trajtory.

The proportional control is adopted to reduce the error.

Problem:
When robot meets a curved traj, there is a delay of steering its direction.

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
ANIMATION_GAP = 20

def plot_traj(traj):
    (x_list, y_list, yaw_list) = traj.query_pose()
    plt.plot(x_list, y_list,'b-o', ms=2)
    plt.plot(xl0, yl0, 'ro',ms=4)

def main_PurePursuit(robot, traj, T):

    dt = 0.1  # control period
    traj.compute_discrete_trajecotry(t_start=0, t_end=T, dt=dt)
    v_mean=traj.total_length/T
    goal_dist=0.2

    # start
    t_curr = robot.query_time()
    idx_near=0 # the index of the nearest point
    path=Path() # record path
    path.append(robot.x, robot.y, robot.yaw)

    for step in range(1, 2000 ):

        # get current pose and goal pose
        (x_curr, y_curr, yaw_curr) = (robot.x, robot.y, robot.yaw)

        # distance to the nearest point -- reduce it
        search_range=20
        idx_l=max(0,idx_near-search_range)# left search index
        idx_r=min(traj.N, idx_near+search_range) # right search index
        dist_near_list=(x_curr-traj.x_list[idx_l:idx_r])**2+\
            (y_curr-traj.y_list[idx_l:idx_r])**2
        idx_near=idx_l+np.argmin(dist_near_list)
        dist_near=np.sqrt(dist_near_list[idx_near-idx_l])
        (x_near, y_near, yaw_near)=traj.query_pose(idx=idx_near)
        # print("x_near=%.2f, y_near=%.2f, yaw_near=%.2f" % (x_near, y_near, yaw_near))
        
        # Angle error
        # ang1: where is the nearest point relatvie to current pose
        ang1 = np.arctan2(y_near-y_curr, x_near-x_curr) 
        ang1_error=angle_pi_to_pi(ang1-yaw_curr)
        if ang1_error<0: # point on the left
            dist_near=-dist_near # w negative
        # ang2: what is the desired orientatino at that nearest point
        ang2=yaw_near
        ang2_error=angle_pi_to_pi(ang2-yaw_curr)


        # Apply PID control
        Pw1=20 # make robot closer to the nearest point
        Pw2=8 # make robot same orientation as the nearest point
        if step==1:
            pid_w=PIDcontroller(
                P=[Pw1, Pw2], D=[0, 0], I=[0, 0], dim=2) # dim=2: 2 params
        w=pid_w.feedback(err=[dist_near, ang2_error])
        v=v_mean

        # output
        robot.move(v, w, dt)
        path.append(robot.x, robot.y, robot.yaw)

        # print current state
        t_curr = robot.query_time()
        print("step=%d, t=%.3f, x=%.2f, y=%.2f, v=%.2f, w=%.2f, yaw=%.2f, theta=%.2f; dist_near=%.2f, ang2_error=%.2f"
                % (step, t_curr, robot.x, robot.y, v, w,
                robot.yaw/np.pi*180, robot.theta/np.pi*180,
                dist_near, ang2_error))

        # check if reach target
        reach_target = False
        if calc_dist(x_curr, y_curr, traj.x_list[-1], traj.y_list[-1])<goal_dist:
            reach_target=True

        # plot
        if (ANIMATION_ON and (step % ANIMATION_GAP == 0)) or reach_target:
            ar=[float(x) for x in [x_curr, y_curr, 0.2*math.cos(robot.yaw), 0.2*math.sin(robot.yaw)]]
            plt.arrow(ar[0],ar[1],ar[2],ar[3],
                head_width=0.08, head_length=0.08, linewidth=4, fc='k', ec='k')
            plot_traj(traj)
            plt.plot(path.xl, path.yl, 'r-,')
            plt.plot(x_near, y_near, 'xr',ms=12,linewidth=2) # next
            plt.plot(x_curr, y_curr, 'ok',ms=5,linewidth=2)  # current 

            plt.pause(0.0001)
            if reach_target:
                mycv.savefig(IMAGE_PATH+"/FollowLine.png")
            plt.show()
        
        if reach_target:
            print "\nReach the target !!!"
            path.plot_velocity(dt, x_axis_time=robot.query_time())
            mycv.savefig(IMAGE_PATH+"/FollowLine_vel.png")
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