
'''
Some common used functions
'''

import numpy as np
import matplotlib.pyplot as plt
import math
import sys

from math import floor, ceil
from numpy import pi

def calc_norm(x1, y1, x2, y2):
    return (x1-x2)**2+(y1-y2)**2
def calc_dist(x1, y1, x2, y2):
    return np.sqrt((x1-x2)**2+(y1-y2)**2)
def sigmoid(x):
    return 1.0/(1+np.exp(-x))
def angle_pi_to_pi(theta):# transform angle to [-pi, pi)
    return (theta+pi) % (2*pi)-pi
def bound(val, thre):
    # cut off val to make it between [-thre, thre]
    return max(min(val, thre), -thre)


def CalcNodeDist(q1,q2):
    try:
        q1=[q1.x,q1.y]
        q2=[q2.x,q2.y]
        return math.sqrt((q1[0]-q2[0])**2+(q1[1]-q2[1])**2)
    except:
        print("CalcNodeDist: the input class must have .x and .y member!")
        sys.exit(0)

def print_node(q):
    print "node x=%.1f, y=%.1f" %(q.x, q.y)