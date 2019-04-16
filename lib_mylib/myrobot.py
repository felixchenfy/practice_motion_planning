
'''
A single-track robot model is defined here.
'''

import numpy as np
import matplotlib.pyplot as plt
import math
import time
import sys


def bound(x, thre):
    thre = abs(thre)
    if x > thre:
        return thre
    elif x < -thre:
        return -thre
    else:
        return x


class PIDcontroller(object):
    def __init__(self, P=0, I=0, D=0, dim=1):
        self.P = np.zeros(dim)+P
        self.I = np.zeros(dim)+I
        self.D = np.zeros(dim)+D
        self.err_inte = np.zeros(dim)
        self.err_prev = np.zeros(dim)
        self.dim = dim

    def feedback(self, err):
        out = 0
        err = np.array(err)

        # P
        out += np.dot(err, self.P)

        # I
        self.err_inte += err
        out += np.dot(self.err_inte, self.I)

        # D
        out += np.dot(err-self.err_prev, self.D)
        self.err_prev = err
        return out

# The robot car with a front wheel and a rear wheel
class RobotModel_SingleTrack(object):
    def __init__(self,
                 # initial pose x, y, and direction yaw (m or rad)
                 x=0.0, y=0.0, yaw=0.0,
                 theta=0,  # initial steering angle (rad)
                 v=0.0,  # initial velocity (m/s)
                 theta_max=math.pi/3, # max steering angle of front wheel (rad)
                 L=0.2,  # distance between the front and rear wheel (m)
                 dt_simulation=0.001,  # low level simulation (s)
                 vmax=0.5,  # max linear velocity of the rear wheel (m/s)
                 dtheta_max=999*math.pi, # max steering rate of front wheel (rad/s)
                 # dtheta_max is current set as Infinite
                 a=1,  # max acceleration of the rear wheel (m/s^2)
                 ):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)  # direction of the robot car

        # the direction of the front wheel relatvie to the rear wheel
        self.theta = float(theta)
        self.v = float(v)  # velocity of the rear wheel
        self.a = float(a)  # max acceleration of the linear velocity
        self.theta_max = theta_max  # max steering angle
        self.vmax = vmax
        self.dtheta_max = dtheta_max

        self.L = float(L)  # length between two wheels
        self.dt_simulation = float(dt_simulation)
        self.time_ = 0.0

        self.v_mean = None # the desired average velocity
        return

    def get_state(self):
        return (self.x, self.y, self.yaw, self.v)

    def get_angular_velocity(self):
        return self.v / self.L * math.tan(self.theta)

    def get_max_angular_velocity_at_v(self, v):
        return v / self.L * math.tan(self.theta_max)

    def reset_time(self):
        self.time_ = 0.0

    def query_time(self):
        return self.time_

    # Input the control command v and w, and try to move the robot for t seconds
    def move(self, v, w, t):
        # rear wheel velocity = v,
        # rotational speed of the whole robot = w
        # front wheel angle = theta

        ite = 0
        v = min(v, self.vmax)  # add constraint
        v = max(v, -self.vmax)
        t_curr = 0

        while t_curr < t and ite < 10000:
            ite += 1
            t_new = min(t, t_curr+self.dt_simulation)
            dt = t_new - t_curr

            # What is theta that gives the desired w?
            # w = v/L*tan(theta) --> theta = arctan(w*L/v)
            if self.v == 0:
                theta = 0
            else:
                theta = math.atan(w * self.L / self.v)

            # set target
            theta_target = bound(theta, self.theta_max)
            v_target=v

            # update value
            v_new, delta_v = self.approach_value(self.v, v_target, self.a, dt)
            theta_new, delta_theta = self.approach_value(self.theta, theta_target, self.dtheta_max, dt)

            self.update_state(delta_v/dt, delta_theta/dt, dt)
            t_curr = t_new

    # ----------------------------------------------------------
    # Simulation of Robot Movement !!!
    # a:        acceleration of the rear wheel
    # dtheta:   angular velocity of the steering of front wheel
    # ----------------------------------------------------------
    def update_state(self, a, dtheta, dt):
        self.theta = self.theta + dtheta * dt
        self.x = self.x + self.v * math.cos(self.yaw) * dt
        self.y = self.y + self.v * math.sin(self.yaw) * dt
        self.yaw = self.yaw + self.get_angular_velocity() * dt
        self.v = self.v+a*dt
        self.time_ += dt

    # try to approach "x_curr" towards "x_goal" by a value of "v*dt"
    def approach_value(self, x_curr, x_goal, v, dt):
        x_curr0 = x_curr
        if x_goal > x_curr:
            x_curr += v*dt
            if x_curr > x_goal:
                x_curr = x_goal
        else:
            x_curr -= v*dt
            if x_curr < x_goal:
                x_curr = x_goal
        delta_x = x_curr-x_curr0
        return x_curr, delta_x

    def check_constraint(self):
        return True

    def sleep(self, t):
        time.sleep(t)
        self.time_ += t


if __name__ == "__main__":

    class Path(object):
        def __init__(self):
            self.xl = list()
            self.yl = list()
            self.yawl = list()

        def append(self, x, y, yaw):
            self.xl.append(x)
            self.yl.append(y)
            self.yawl.append(yaw)

    # set robot
    robot = RobotModel_SingleTrack(
        x=0, y=0, yaw=math.pi/2,  # pose of robot
        theta=0,  # steering angle of the front wheel
        v=0,  # velocity of the rear wheel
        L=0.2,  # dist between front and rear wheel
        dt_simulation=0.01,  # low level simulation for updating robot states
        theta_max=math.pi/4,  # max steering angle of the front wheel
        vmax=0.5
    )

    path = Path()
    path.append(robot.x, robot.y, robot.yaw)
    for i in range(50):
        robot.move(0.04, 2.5, 0.1)
        path.append(robot.x, robot.y, robot.yaw)
    plt.plot(path.xl, path.yl)
    plt.show()
