#!/usr/bin/env python2
# -*- coding: utf-8 -*-

'''
A trajectory class for commonly used operations on trajectory points.
See descriptions below "class Trajectory(object)"
'''



import math
import time
import sys
import numpy as np
from numpy import sqrt, pi, sin, cos, arctan2
from myDS import BinarySearch
import matplotlib.pyplot as plt

# The path is used for storing the poses of a robot.
# It has three elements: list of x, list of y, and list of yaw
class Path(object):
    def __init__(self):
        self.xl, self.yl, self.yawl  = list(), list(), list()

    def append(self, x, y, yaw):
        self.xl.append(x)
        self.yl.append(y)
        self.yawl.append(yaw)

    def append_list(self, xl, yl, yawl):
        for i in range(len(xl)):
            self.xl.append(xl[i])
            self.yl.append(yl[i])
            self.yawl.append(yawl[i])

    def compute_derivative(self, dt=1):
        xl = np.array(self.xl)
        yl = np.array(self.yl)
        yawl = np.array(self.yawl)
        dxl = (xl[1:]-xl[0:-1])/dt
        dyl = (yl[1:]-yl[0:-1])/dt
        dyawl = (yawl[1:]-yawl[0:-1])/dt
        # dvl=[sqrt(dx**2+dy**2) for (dx,dy) in (dxl, dyl)]
        vl = np.sqrt(dxl**2+dyl**2)
        return (dxl, dyl, dyawl, vl)

    def plot_velocity(self, dt, x_axis_time=1, flag_plt_show=False):
        plt.figure(figsize=(12,6))
        plt.subplot(121)
        (dxl, dyl, dyawl, vl)=self.compute_derivative(dt)
        t=1.0*np.arange(len(dxl))/len(dxl)*x_axis_time
        plt.plot(t, vl,'r-')
        plt.xlabel("time (s)")
        plt.ylabel("veloicty (m/s)")
        plt.subplot(122)
        plt.plot(t, dyawl,'b-')
        plt.xlabel("time (s)")
        plt.ylabel("angular velocity (rad/s)")
        if flag_plt_show:
            plt.show()


class Trajectory(object):

# The Trajectory class reads in the 
# time-depend trajecotry function {x(t), y(t)} of the robot,
# and then provides some common operations on them.
# * compute pose (x, y, yaw) at a given time t
# * compute a list of poses of descrete trajectory points: self.x_list, self.y_list, etc
# * compuate derivative of the traj at a given time using numerical method

# It also provides the length of the path wrt time: s(t)=∫(dx^2+dy^2)dt
# You can query the time td that makes s(td)=Sd. The func will return a approximate time.
# You can also do the TIME_SCALING, which changes the values of self.t_list / x_list / y_list,
#   to make all neighbor points {x(ti),y(ti)} and {x(ti+1),y(ti+1)} have a distance of veloicty*dt_new

    # init trajectory with its function x=x(t) and y=y(t) with param
    def __init__(self, fxt=None, fyt=None, param=None, t_start=0, t_end=None, dt=None):
        self.set_time(t_start, t_end, dt)
        self.set_function(fxt, fyt, param)
        self.delta_t = 1e-6  # used for numerically compute derivative
        # this is to deal with the discontinuity of yaw = atan2(dy,dx) when numerically compute the d(yaw)/d(t)
        self.MAX_CHANGE = 2.0/3*pi

    def set_time(self, t_start, t_end, dt):
        self.t_start = t_start  # start time of a trajectory. Defualt 0.
        self.t_end = t_end  # end time of the trajectory
        self.dt = dt
        if t_end is not None and dt is not None:
            self.N = int((self.t_end-self.t_start)/dt)  # number of target points

    # set x=x(t) and y=y(t)
    def set_function(self, fxt, fyt, param):
        self.param = param
        if fxt is not None:
            if param is None:
                self.fxt = lambda t: fxt(t)
                self.fyt = lambda t: fyt(t)
            else:
                self.fxt = lambda t: fxt(t, self.param)
                self.fyt = lambda t: fyt(t, self.param)
        else:
            self.fxt = None
            self.fyt = None

    # derivative of f at t0
    def derivative(self, f, t0):
        delta_t = self.delta_t
        return (f(t0+delta_t)-f(t0))/delta_t

    # -------------- compute pose at a single time t ----------------

    def calc_x(self, t):
        return self.fxt(t)

    def calc_dx(self, t):
        return (self.fxt(t+self.delta_t)-self.fxt(t))/self.delta_t

    def calc_y(self, t):
        return self.fyt(t)

    def calc_dy(self, t):
        return (self.fyt(t+self.delta_t)-self.fyt(t))/self.delta_t

    def calc_yaw(self, t):
        y_next = self.fyt(t+self.delta_t)
        y_curr = self.fyt(t)
        x_next = self.fxt(t+self.delta_t)
        x_curr = self.fxt(t)
        return np.arctan2(y_next-y_curr, x_next-x_curr)

    def calc_dyaw(self, t):
        yaw_next = self.calc_yaw(t+self.delta_t)
        yaw_curr = self.calc_yaw(t)
        # cope with the sudden change of yaw from +pi to -pi
        if yaw_next > yaw_curr + self.MAX_CHANGE:
            yaw_next -= 2*pi
        if yaw_next < yaw_curr - self.MAX_CHANGE:
            yaw_next += 2*pi
        return (yaw_next-yaw_curr)/self.delta_t

    # -------------- compute descrete trajectory values --------------

    # compute trajectory between [t_start, t_end]
    # (x, y, yaw) are evaluated at [0, dt, 2*dt, ..., t_end]
    def compute_discrete_trajecotry(self, t_start, t_end, dt, flag_do_integral=True, flag_do_derivative=True):
        self.set_time(t_start, t_end, dt)
        self.t_list = np.linspace(t_start, t_end, self.N+1)

        # (x, y, yaw)
        self.x_list = self.fxt(self.t_list)
        self.y_list = self.fyt(self.t_list)
        self.yaw_list = self.compute_yaw_list()

        # (dx, dy, dyaw)
        if flag_do_derivative:
            self.dyaw_list = self.compute_dyaw_list()
            self.dx_list = self.derivative(self.fxt, self.t_list)
            self.dy_list = self.derivative(self.fyt, self.t_list)
            self.v_list = np.sqrt(self.dx_list**2+self.dy_list**2)

        # integrated_path: s(t).
        # s(ti) is the length of path that robot moves during [t_start, ti]
        if flag_do_integral:
            segment_length = np.sqrt((self.x_list[0:-1]-self.x_list[1:])**2 +
                                     (self.y_list[0:-1]-self.y_list[1:])**2)  # N segments
            integrated_path = [0, ]
            for i in range(1, self.N+1):
                integrated_path.append(integrated_path[-1]+segment_length[i-1])
            # self.total_length=np.sum(self.segment_length)
            self.integrated_path = integrated_path
            self.total_length = integrated_path[-1]

    def compute_discrete_trajecotry_with_constant_v(
        self, t_start, t_end, dt, v, prec_ratio):
        self.compute_discrete_trajecotry(t_start, t_end, dt*v*prec_ratio, flag_do_integral=True, flag_do_derivative=False)
        self.TIME_SCALING(v, dt)
        
        # Now fxt and fyt have been changed.
        # Functions related to self.fxt and self.fyt are no longer supported.
        self.integrated_path=None # clear

    # find the time t, so that s(t)=S. Return the t, and x and y at that moment
    # s is the length of the path from the start point to the time t
    def query_xyt_of_path_point(self, S):
        ind, val = BinarySearch(self.integrated_path, S)
        t = self.t_list[ind]
        return self.x_list[ind], self.y_list[ind], t, ind

    # The original traj is {x(t), y(t)}, where the robot sometimes moves fast, sometimes slow.
    # After scaling, the robot will move at a constant velocity (or other velocity curve if I write it in future version).
    # In other words, the descrete points {x(ti), y(ti)} on the traj now have same distance between the two.
    # NOTE:
    #   * This funcs will modify the original traj !!!
    #   * after time scaling, the list dx and dy, and a lot of other operations can no longer be used
    #   * This requires a high resolution of integration steplength. The original dt should be << dt_new (such as 0.01 versus 0.1)
    def TIME_SCALING(self, velocity, dt_new):
        tlist = [self.t_start, ]
        xlist = [self.x_list[0], ]
        ylist = [self.y_list[0], ]
        t = 0
        cnt = 0
        while 1:
            cnt += 1
            t = t+dt_new
            s_desired = velocity*t
            (x, y, t_old, ind) = self.query_xyt_of_path_point(s_desired)
            tlist.append(t+self.t_start)
            xlist.append(x)
            ylist.append(y)
            if ind == len(self.integrated_path)-1: # reach the end
                break
        self.t_list, self.x_list, self.y_list = tlist, xlist, ylist
        self.t_end = t
        self.N = cnt
        self.dt = dt_new

    # compuate yaw and dyaw
    def compute_yaw_list(self):
        delta_t = self.delta_t
        y_1 = self.fyt(self.t_list+delta_t)
        x_1 = self.fxt(self.t_list+delta_t)
        yaw_list = np.arctan2(
            y_1-self.y_list,
            x_1-self.x_list
        )
        return yaw_list

    def compute_dyaw_list(self):
        delta_t = self.delta_t

        y_1 = self.fyt(self.t_list+delta_t)
        x_1 = self.fxt(self.t_list+delta_t)
        yaw_0 = np.arctan2(
            y_1-self.y_list,
            x_1-self.x_list
        )

        y_2 = self.fyt(self.t_list+delta_t*2)
        x_2 = self.fxt(self.t_list+delta_t*2)
        yaw_1 = np.arctan2(
            y_2-y_1,
            x_2-x_1,
        )
        delta_yaw = yaw_1 - yaw_0

        # cope with the sudden change of yaw from +pi to -pi
        delta_yaw[delta_yaw > self.MAX_CHANGE] -= 2*pi
        delta_yaw[delta_yaw < -self.MAX_CHANGE] += 2*pi

        # compuate dyaw
        dyaw_list = delta_yaw/delta_t
        return dyaw_list

    # -------------- query descrete trajectory values --------------

    def query_para(self):
        return (self.t_end, self.N, self.dt)

    def query_pose(self, t=None, idx=None):
        if idx is None and t is None:
            return (self.x_list, self.y_list, self.yaw_list)
        elif idx is not None:
            return (self.x_list[idx], self.y_list[idx], self.yaw_list[idx])
        elif t is not None:
            return (self.calc_x(t), self.calc_y(t), self.calc_yaw(t))

    def query_dpose(self):
        return (self.dx_list, self.dy_list, self.dyaw_list)


if __name__ == "__main__":
    dt = 0.1
    # set path points
    if 1:  # this is shorter, for testing
        xl0 = [10, 6, 5, 2]
        yl0 = [5, -3, 1, 1]
        xl0 = np.array(xl0)/5.0
        yl0 = np.array(yl0)/5.0
        T = 8  # total time (s)

    import matplotlib.pyplot as plt
    from myspline import Spline

    # init trajectory
    # spline a function for the path
    spline = Spline(xl0, yl0, T, method="CubicSpline")

    # assign spline to trajectory
    N = int(T/dt)
    traj = Trajectory(spline.fxt, spline.fyt, param=None)
    traj.compute_discrete_trajecotry(t_start=0, t_end=T, dt=dt)

    # plot the trajectory after interpolation
    plt.figure(figsize=(18, 5))
    plt.subplot(131)
    plt.plot(traj.x_list, traj.y_list, 'g.')
    plt.plot(xl0, yl0, 'rx')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Trajectory after interpolation")

    plt.subplot(132)
    plt.plot(traj.t_list, traj.yaw_list, 'r')
    plt.plot(traj.t_list, traj.dyaw_list, 'g')
    plt.legend(("yaw", "d_yaw"))
    plt.title("Robot yaw angle after interpolation")

    # ------------------------
    # do time scaling
    # its output trajectory will have a uniformly distributed points
    if 0:
        traj.TIME_SCALING(velocity=1.0, dt_new=0.1)
    else:
        traj.compute_discrete_trajecotry_with_constant_v(
            t_start=0, t_end=T, dt=0.1, v=0.5, prec_ratio=0.1)

    plt.subplot(133)
    plt.plot(traj.x_list, traj.y_list, 'g.')
    plt.plot(xl0, yl0, 'rx')
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Time scaling")

    plt.show()
