#!/usr/bin/env python2
# -*- coding: utf-8 -*-

'''
------------------------------------------------------------------------------------
this is a class and an example of using non-linear optimization
for tracking a given trajectory.

The cost function is the distance between robot current pose and 
    a goal point (pose) on the reference trajecotry.
    
The 5 variables to optimize are:
  * v, w: Control input of the desired linear and angular velocity
    for the robot (which might not be achieved).
  * x, y, yaw: current pose of robot.

Constraints:  
  * The relationship between (v,w) and (x,y,yaw) satisfies the 1st-order integration.
  * The w is smaller than the w_max, where w_max is subject to v 
    and max steering angle. Without this, the trajectory of
    what optimization thinks will be different from the real one.
  * Constrain on the acceleration of linear velocity.

I wrote this code after reading:
Prof Kevin Lynch's book: Modern Robotics
Chapter 10.7 Nonlinear Optimization.

There is no explicit solution in the book, so what I write is only based on
    my own understanding, which is definitely not so correct.
Actually, I've met two difficulties when using optimization method,
    and I feel difficult to solve them unless I make big changes to the whole script.
    Someday, I might read others' paper, and modify my code.

* Car model in simulation:
The car model used for simulation is "Single Track".
I set it's lowest-level-control input as "a" and "dtheta", where:
a:       acceleration of the rear wheel
dtheta:  steering speed (changing of direction) of the front wheel

* Car model in optimization:
Since
    1. I made the robot accepting the control input of (v, w), 
    2. and it's a bit complex to optimize on (a, dtheta),
the parameters waiting to be optimized are (v, w),
which refers to a "Differential Drive" model. 

I've tried 3 different simplified versions of  "Differential Drive" model
for the optimizer to mimic the dynamics. This one works the best:
        yaw=yaw+dt*w
        x=x+dt*v*cos(yaw)
        y=y+dt*v*sin(yaw)


## Path Tracking Result and Problems
1. When the future planning time is small, 
it's better than PID based PurePursuit.py or FollowLine.py (without feedforward) when making small turns,
because this method can see (a little bit of) future.

2. When planning for a longer time (increasing "opti_length" in my code), 
theoratically it should be improve the performance.
    However, there appears some strange loops on the resultant path.
This is due to the cubic spline method produces dense points in high curved places.
For Single-Track robot, low linear speed means low angular speed. 
The robot gets really difficult to traverse this high curved line with low speed,
    so it decides to make a "loop". Very strange.

I tried using traj.TIME_SCALING to get "fixed distance points"
    to replace the original "points from cubic spline",
    but the performance goes worth.

Solution: I think the correct solution should be changing the cost function,
from "minimizing distance from robot to target point",
to "minimizing distance from robot to the trajectory, as well as minimizing time cost".

I should really read some papers before keep on modifying this code.
(Commented on 2019 Jan. 4th: I think I won't choose motion planning as my master's project.)

## Other notes
1. This method relies highly on a good initial guess

2. Time cost of optimization is non-linear to the number of points N,
    may be something like N^2, I don't know.
So optimizing {a few points , like 5 or 10, for several times}
    is much faster than {30 or 40 for once}.

------------------------------------------------------------------------------------
'''


'''
------------------------------------------------------------------------------------
General formula of non-linear optimization
------------------------------------------------------------------------------------
find u(t), q(t), T
minimizing J(u(t), q(t), T)
x_dot(t)=f(states(t),u(t))
states(0)=x_start
states(T)=x_goal

------------------------------------------
symbols' meaning
------------------------------------------
u: control input, linear and angular velocity (v, w)
q: trajectory. Here on a plane, q=(X, Y, yaw)
T: Total time
x: state of the robot. Here x=(X, Y, yaw)
J: Cost function
   the pose q at time ti should be 
   equal to the desired pose q* 
   Example:
       J=sigma( ||q(ti)-q*(ti)||^2 )
       q*(t)=(X*(t), y*(t), yaw*(t))

------------------------------------------
# State transition
# Input:  state x; control input u=(v, w)
# Output: state changing rate x_dot

A simplify example for the state transition looks like this:
x_dot=(X_dot, Y_dot, yaw_dot)
    where:
    X_dot=v*cos(yaw)
    Y_dot=v*sin(yaw)
    yaw_dot=w

------------------------------------------------------------------------------------
'''

import numpy as np
import matplotlib.pyplot as plt
import math

import os
import sys
FILE_PATH = os.path.join(os.path.dirname(__file__))
IMAGE_PATH = FILE_PATH+'/../images'
sys.path.append(FILE_PATH + "/../lib_mylib")

from scipy.optimize import minimize
from myrobot import PIDcontroller, RobotModel_SingleTrack
from myspline import Spline
from mytrajectory import Trajectory, Path
from mycv import mycv

IF_ANIMATION = False
flag_path_CubicSpline = True
PATH_SameDistance = not flag_path_CubicSpline

class PlanByOptimization(object):
    def __init__(self):
        return

    def optimize(self, curr_state, traj, method_ID=0,
                 max_opt_pts=40, states0=None, print_detials=True,
                 robot=None):
        # input:
        #   curr_state: type "np.array((5,1))"
        #       v0, w0, x0, y0, yaw0
        #   traj: type "Trajectory"
        #       the trajectory for the robot to follow
        #       member required: x_list, y_list, N, T, dt
        #
        #   max_opt_pts: max number of poses to optimize at one time
        #   states0: a initial guess of all states
        #
        # output:
        #   states_res: optimized states
        #       type: np.array([[v], [w], [x], [y], [yaw]])
        #       v, w: control input of (v, w)
        #       x, y, yaw: robot pose

        states_dim = 5
        states_res = np.zeros((states_dim, 0))

        self.curr_state = curr_state
        self.dt = traj.dt
        self.method_ID = method_ID
        if robot is None:
            print("\nIn this version, you have to input a robot model.")
        self.robot=robot
        self.print_detials = print_detials

        num_segments = int(np.ceil(1.0*traj.N/max_opt_pts))
        for i in range(num_segments):
            # extract the ith segment to optimize
            idx_1 = i*max_opt_pts+1
            idx_f = min((i+1)*max_opt_pts, traj.N)
            # notics, traj includes the starting point,
            #   so the index start from 1
            self.x_list = traj.x_list[idx_1: idx_f+1]
            self.y_list = traj.y_list[idx_1: idx_f+1]
            self.yaw_list = traj.yaw_list[idx_1:idx_f+1]
            self.N = len(self.x_list)

            # initial guess of all states
            if states0 is None:
                N = self.N  # set as the desired trajectory
                self.states0 = np.random.random((states_dim, N))*0.001
                if flag_path_CubicSpline:
                    self.states0[0, :] += traj.v_list[idx_1: idx_f+1]
                    self.states0[1, :] += traj.dyaw_list[idx_1: idx_f+1]
                else:
                    self.states0[0, :] += robot.v_mean
                self.states0[2, :] = self.x_list
                self.states0[3, :] = self.y_list
                self.states0[4, :] = self.yaw_list

            else:
                self.states0 = states0[:, idx_1-1: idx_f]
                # you can put the output of the last execution
                #   as the init states. Though there is little effect.
            # print i, idx_1, idx_f, self.states0.shape[1]

            # optimize
            if print_detials:
                print("\n------------------------------------------------")
                print("Progress %d / %d ..." % (i*max_opt_pts, traj.N))
                print("start optimize, please wait for 5 seconds ...")
            
            (states, cost) = self._optimize()
            
            if print_detials:
                print("Complete! cost =", cost)
                print("------------------------------------------------")

            # output list
            states_res = np.hstack([states_res, states])

            # update current pose
            self.curr_state = states[:, -1:]

        (v, w) = (states_res[0, :], states_res[1, :])
        (x, y, yaw) = (states_res[2, :], states_res[3, :], states_res[4, :])
        return (v, w, x, y, yaw, states_res)

    def _optimize(self):

        ### input args
        N = self.N
        dt = self.dt
        curr_state = self.curr_state

        xd_list = self.x_list  # desired x_list
        yd_list = self.y_list
        yawd_list = self.yaw_list

        ### Bounds: bnds
        #   bounds for all states

        vmax=self.robot.vmax
        wmaxmax=self.robot.get_max_angular_velocity_at_v(v=vmax)
        # note that, the max of wmax (noted as wmaxmax) is achieved at vmax.
        # if v < vmax, then wmax < wmaxmax.

        bnds_v = [(-vmax, vmax)]*N
        bnds_q = [(-wmaxmax,wmaxmax)]*N
        bnds_xyyaw = [(-9999, 9999)]*3*N  # no bounds for pose

        bnds_list = [bnds_v, bnds_q, bnds_xyyaw]
        bnds = list()
        for b in bnds_list:
            bnds = bnds+b
        bnds = tuple(bnds)

        ### CostFunc: cost_func
        #   cost function needs to be minimized

        def cost_func(states):  # states is a 5*N ndarray
            states = states.reshape((-1, N))
            (x, y, yaw) = self.get_xyyaw(states)

            diff_x = x - xd_list
            diff_y = y - yd_list
            diff_yaw=yaw-yawd_list

            diff_yaw[diff_yaw> math.pi]-=2*math.pi
            diff_yaw[diff_yaw<-math.pi]+=2*math.pi

            weight_yaw=0.05**2
            weight_time=np.linspace(1,1,N)
            # tmp1x=(np.cos(yaw)-np.cos(yawd_list))**2
            # tmp2y=(np.sin(yaw)-np.sin(yawd_list))**2
            # diff_yaw=np.sqrt(tmp1x+tmp2y)*weight  

            err_x=diff_x**2 *weight_time
            err_y=diff_y**2 *weight_time
            err_yaw=diff_yaw**2*weight_yaw *weight_time
            
            return np.sum(err_x+err_y+err_yaw)

        ### Constraints:
        #   cons_differential_model: Eq cons
        #       here simplify as: x(t+dt)==x+v*dt
        #   cons_acceleration: Ineq cons
        #       acceleration should not be not too small or large

        def differential_model(x0, y0, yaw0, v, w):
            phi = w * dt
            l = v * dt
            r = l / phi
            # the next pose in car's frame
            xc = r*np.sin(phi)
            yc = r-r*np.cos(phi)
            # the next pose in world frame
            c = np.cos(yaw0)
            s = np.sin(yaw0)
            xw = c*xc-s*yc+x0
            yw = s*xc+c*yc+y0
            yaww = yaw0+phi
            return (xw, yw, yaww)

        def cons_differential_model(states):
            states = states.reshape((-1, N))
            st = np.hstack([curr_state, states])  # states
            (v1, w1) = self.get_vw(st, 1, N+1)
            (x1, y1, yaw1) = self.get_xyyaw(st, 1, N+1)
            (x0, y0, yaw0) = self.get_xyyaw(st, 0, N)
            idx_switch=1
            if idx_switch==1:  # simple model: rotate and then move straight
                yaw1_p = yaw0 + dt * w1
                x1_p = x0 + dt * v1 * np.cos(yaw1_p)
                y1_p = y0 + dt * v1 * np.sin(yaw1_p)
            elif idx_switch==2: # This is really bad ### needs to check why
                x1_p = x0 + dt * v1 * np.cos(yaw0)
                y1_p = y0 + dt * v1 * np.sin(yaw0)  
                yaw1_p = yaw0 + dt * w1             
            else:  # differential model. Bad performance.
                (x1_p, y1_p, yaw1_p) = differential_model(x0, y0, yaw0, v1, w1)
                None
            # constraint
            c1 = x1 - x1_p  # true x == predict x based on model
            c2 = y1 - y1_p
            c3 = yaw1 - yaw1_p
            return list(c1)+list(c2)+list(c3)

        def cons_angular_velocity(states): 
            # this is needed for Single Track robot model, because w_max depends on v.
            # Not needed for Differential Driver, becaue w_max doesn't depend on v.

            st = states.reshape((-1, N))
            (v, w) = self.get_vw(st)
            wmax=self.robot.get_max_angular_velocity_at_v(v)

            # add constraint
            c1 = wmax - w # > 0
            c2 = w - (-wmax) # > 0
            return list(c1)+list(c2)

        ### cons on acceleration
        def cons_acceleration(states):
            st=states.reshape((-1,N))
            st=np.hstack([curr_state, st]) # states
            (v0, w0) = self.get_vw(st, 0, N)
            (v1, w1) = self.get_vw(st, 1, N+1)
            dv = (v1 - v0)/dt
            dv_max = self.robot.a
            c1 = dv_max - dv
            c2 = dv - (-dv_max)
            return list(c1)+list(c2)

        cons = ([{'type': 'eq', 'fun': cons_differential_model},
                  {'type': 'ineq', 'fun': cons_acceleration},
                  {'type': 'ineq', 'fun': cons_angular_velocity},
                 ])

        ### Solve
        # choose a method from below
        methods = ["SLSQP",  # bound + cons, very very slow
                   "COBYLA",  # ineq cons only.
                   "TNC",  # bound only.
                   "L-BFGS-B"]  # bound  only.

        # https://docs.scipy.org/doc/scipy-0.18.1/reference/optimize.minimize-slsqp.html#optimize-minimize-slsqp

        # NOTE: "minimize" automatically changes states to a row vector
        solution = minimize(cost_func, self.states0,
                            method=methods[self.method_ID],
                            bounds=bnds, constraints=cons,
                            options={'maxiter': 200, 'disp': False})
        states = solution.x
        cost = cost_func(states)
        return (states.reshape((-1, N)), cost)

    # define two funcs for extracting data
    def get_vw(self, states, l=None, r=None):
        if l is None:
            (v, w) = (states[0, :], states[1, :])
        else:
            (v, w) = (states[0, l:r], states[1, l:r])
        return (v, w)

    def get_xyyaw(self, states, l=None, r=None):
        if l is None:
            (x, y, yaw) = (states[2, :], states[3, :], states[4, :])
        else:
            (x, y, yaw) = (states[2, l:r], states[3, l:r], states[4, l:r])
        return (x, y, yaw)


def main():
    dt = 0.1  # control period

    # set path points
    if 0: # this is shorter, for testing
        xl0 = [10, 6, 5, 2]
        yl0 = [5, -3, 1, 1]
        xl0 = np.array(xl0)/5.0
        yl0 = np.array(yl0)/5.0
        T = 15.0  # total time (s)
    else: # this is longer
        xl0 = [0, 1, 5, 9, 11, 10, 6, 5, 2]
        yl0 = [0, 3, 5, -1, -4, 5, -3, 1, 1]
        xl0=np.array(xl0)/5.0
        yl0=np.array(yl0)/5.0
        T = 30.0 # total time (s)

    # spline a function for the path
    spline = Spline(xl0, yl0, T)
    traj = Trajectory(spline.fxt, spline.fyt, param=None)# assign spline to trajectory 
    traj.compute_discrete_trajecotry(0, T, dt)

    # set robot
    robot = RobotModel_SingleTrack()
    print("\nMax velocity of the robot: v=%.2f, w=%.2f" %\
         (robot.vmax, robot.get_max_angular_velocity_at_v(robot.vmax)))
    robot.v_mean=traj.total_length/T

    # set optimization parameters
    opti_dt = 0.5       # run optimization for every opti_dt seconds
    opti_length = 1.0   # how long (in seconds) to optimize future traj in each iteration
    MAX_OPT_POINTS_PER_LOOP = int(opti_length/dt) # max number of points to optimize in a low level loop 
                    # in side my optimization function.
                    # Originally I added this functionality in order to speed up the optimization 
                    # by planning segment by segment.
                    # However, it's functionality is the same as setting "opti_dt" and "opti_length" here.
                    # So currently, just set its value as "opti_length/dt".
    opt = PlanByOptimization()

    ### some data history to record
    path=Path() # true path
    path_opti_thought=Path() # what the optimizer thought about the robot's pose
    path.append(robot.x, robot.y, robot.yaw)
    path_opti_thought.append(robot.x, robot.y, robot.yaw)
    vl_input=[]
    wl_input=[]

    ### start robot !!!
    idx_control_commands=0
    ite=-1
    opti_dt_cnt=0
    while robot.query_time()<T:
        t_curr=robot.query_time()
        ite+=1
        if ite == 0 or (opti_dt_cnt>=opti_dt/dt) or len(vl_input_k)==0: # run opti for every opti_dt seconds
            opti_dt_cnt=0
            if flag_path_CubicSpline:
                traj.compute_discrete_trajecotry(
                    t_start=t_curr, t_end=t_curr+opti_length, dt=dt
                )
            else:
                traj.compute_discrete_trajecotry_with_constant_v(
                    t_start=t_curr, t_end=t_curr+opti_length, dt=dt,
                    v=robot.v_mean, prec_ratio=0.1
                )

            curr_state=np.array([[
                robot.v, robot.get_angular_velocity(),
                robot.x, robot.y, robot.yaw
            ]]).T

            (vl_input_k, wl_input_k,
                xl_res_k, yl_res_k, yawl_res_k, states_res) =\
                opt.optimize(curr_state, traj, method_ID=0,
                            max_opt_pts=MAX_OPT_POINTS_PER_LOOP, print_detials=False,
                            robot=robot)

            vl_input_k=list(vl_input_k)
            wl_input_k=list(wl_input_k)
            path_opti_thought.append_list(xl_res_k, yl_res_k, yawl_res_k)        
            
            if IF_ANIMATION:
                plt.clf()
                plt.plot(xl_res_k, yl_res_k,'gx')
                # plt.plot(traj.x_list, traj.y_list,'bo-')
                # plt.plot(path_opti_thought.xl, path_opti_thought.yl, 'gx-')  # What optimizer thought
                plt.plot(path.xl, path.yl, 'ro-')  # real traj
                plt.plot(traj.x_list, traj.y_list, 'b.-',linewidth=1)  # desired traj

                plt.legend(["optimizer thought", "real,","desired traj"])
                plt.pause(0.0001)
                plt.show()
        opti_dt_cnt+=1
        # read in the control input
        v=vl_input_k.pop(0)
        w=wl_input_k.pop(0)
        idx_control_commands+=1

        # move robot
        robot.move(v, w, dt)
        print("ite=%d, x=%.2f, y=%.2f, yaw=%.2f; v=%.2f, w=%.2f"\
            %(ite, robot.x, robot.y, robot.yaw, v, w))

        path.append(robot.x, robot.y, robot.yaw)
        vl_input.append(v)
        wl_input.append(w)

    ### plot trajectory
    if flag_path_CubicSpline:
        traj.compute_discrete_trajecotry(t_start=0, t_end=T, dt=dt)
    else:
        traj.compute_discrete_trajecotry_with_constant_v(
            t_start=0, t_end=T, dt=dt,
            v=robot.v_mean, prec_ratio=0.1
        )

    fig = plt.figure(figsize=(10, 7))  # set figure
    plt.plot(path_opti_thought.xl, path_opti_thought.yl, 'gx')  # What optimizer thought
    plt.plot(path.xl, path.yl, 'ro-')  # real traj
    plt.plot(traj.x_list, traj.y_list, 'b.-',linewidth=1)  # desired traj

    plt.title("trajectory")
    plt.legend(["What optimizer thought", "real traj", "desired traj"])
    plt.xlabel("x (m)")
    plt.ylabel("y (m) ")

    mycv.savefig(IMAGE_PATH+"/NonlinearOptimization.png")

    ### plot velocity

    (_, _, wl_real, vl_real) = path.compute_derivative(dt=dt)
    t=np.arange(traj.N)*1.0/traj.N*T+traj.dt

    fig = plt.figure(figsize=(12, 6))
    plt.subplot(121)
    plt.plot(t, vl_real, 'r-.')
    plt.plot(t, vl_input, 'g-.')
    plt.legend(["v(real)", "v(input)"])
    plt.xlabel("t (s)")
    plt.ylabel("v (m/s)")

    plt.subplot(122)
    plt.plot(t, wl_real, 'r-.')
    plt.plot(t, wl_input, 'g-.')
    plt.legend(["w(real)", "w(input)"])
    plt.xlabel("t (s)")
    plt.ylabel("w (rad/s)")

    mycv.savefig(IMAGE_PATH+"/NonlinearOptimization_vel.png")

    plt.show()


if __name__ == "__main__":
    main()
