#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
RRT* path planning algorithm.
You can set USING_RRT_STAR=0 to switch to RRT algorithm.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import math
import copy
from pylab import *
import cv2

import os, sys
FILE_PATH=os.path.join(os.path.dirname(__file__))
IMAGE_PATH=FILE_PATH+'/../images'
sys.path.append(FILE_PATH + "/../lib_mylib")
from mycv import mycv
from mylib import CalcNodeDist, print_node, sigmoid, calc_norm, calc_dist

class Node(object): # The node, or vertex, for the RRT tree.
    # the nodes (vertices) are pushed into a python List
    def __init__(self,x=-1,y=-1,idx=-1,idx_parent=-1,cost=0):
        self.x=x
        self.y=y
        self.idx=idx # the index of this node in the list
        self.idx_parent=idx_parent # the index of the node's parent
        self.cost=cost
        return

class RRT(object):
    PATH_FIND=0

    def __init__(self):
        return

    def init(self,img,q_start,q_target, expand_dis=5,
        dis_goal=3, goal_sample_rate=0.1, max_iter=500):

        self.img=img # obstacle=0, free=1
        self.shape=img.shape

        q_start.idx=0
        q_start.idx_parent=-1
        self.q_list=[q_start]
        self.q_target=q_target

        self.expand_dis=expand_dis
        self.dis_goal=dis_goal # stop when dis<dis_goal
        self.goal_sample_rate=goal_sample_rate
        self.max_iter=max_iter

    def Plan(self):
        find_target=False
        for steps in range (self.max_iter):
            q_rand=self.RandNode()
            q_near=self.FindNearestNode(q_rand)
            q_new=self.NewNode(q_near, q_rand, self.expand_dis)
            
            if q_new is None:
                continue
        
            # RRT_star. Choose a new parent from the neibor nodes
            near_inds = self.FindNearNodes(q_new)
            
            USING_RRT_STAR=1
            if USING_RRT_STAR:
                q_new = self.UpdateParent(q_new, near_inds)
                self.AppendNode(q_new)
                self.Rewire(q_new, near_inds) # the neibor noods might also change parent
            else:
                self.AppendNode(q_new)
                
            if self.IsNearTarget(q_new):
                find_target=True
                break
            
        return (find_target, steps+1)

    def RandNode(self):
        gsr=self.goal_sample_rate
        rate=len(self.q_list)/1000.0
        gsr=gsr*(1+sigmoid(rate)) # the rate to goal increases with sampling times
        if np.random.rand(1)[0]<1-gsr:
            xy=np.multiply(np.random.random(2), [self.shape[0],self.shape[1]]) # randomly pick a position
        else:
            xy=[self.q_target.x,self.q_target.y] # pick the goal,
                # so searching moves towards the goal
        q_rand=Node(xy[0],xy[1],idx=-1,idx_parent=-1)
        return q_rand

    def FindNearestNode(self,q_rand):
        # compare the distance between q_rand and all the nodes in q_list,
        # and then find the nearest node
        min_idx=0
        min_dis=9999999
        for i in range(len(self.q_list)): # iterate through rows
            q=self.q_list[i]
            dis=CalcNodeDist(q,q_rand)
            if dis<min_dis:
                min_dis=dis
                min_idx=i
        q_near=self.q_list[min_idx]
        return q_near
    
    def FindNearNodes(self, q_new):
        d_list=[ calc_norm(q_new.x, q_new.y, q.x, q.y) for q in self.q_list]
        
        nnode=len(self.q_list)
        # search neibors. they might be the new father of q_new
        r = 50.0 * math.sqrt((math.log(nnode) / nnode)) # ≈10~15
        # r=self.expand_dis * 2 
        
        d_list=np.array(d_list)
        near_inds=d_list<r ** 2
        
        near_inds = []
        for ind, dis in enumerate(d_list):
            near_inds.append(ind) if dis <= r ** 2 else None
            
        return near_inds
    
    def NewNode(self,q_near, q_rand, dq):            
        # constrain 1: the new node should within a range of [dq]
        dis=CalcNodeDist(q_near,q_rand)
        if dis<dq:
            q_new=Node(q_rand.x,q_rand.y)
        else:# find a point that is on the line, and has a distance dq
            percent=dq/dis
            x=q_near.x+(q_rand.x-q_near.x)*percent
            y=q_near.y+(q_rand.y-q_near.y)*percent
            q_new=Node(x,y)
                
        # constrain 2: check collision
        # move from q_near to q_new, if meet with a black dot, break it
        (no_collision, x, y)=self.WalkUntilCollision(q_near, q_new)
        
        if calc_dist(x, y, q_near.x, q_near.y)<2: # too near
            return None
        else: # return the new node
            q_new=Node(x,y,len(self.q_list),q_near.idx)
            q_new.cost=q_near.cost+CalcNodeDist(q_new, q_near)
            return q_new
        
    def WalkUntilCollision(self, q_near, q_new):
        step_len=CalcNodeDist(q_near,q_new)
        theta=math.atan2(q_new.y-q_near.y,q_new.x-q_near.x)
        dx=math.cos(theta)
        dy=math.sin(theta)
        x=q_near.x
        y=q_near.y
        step=0
        no_collision=True
        while step<step_len:
            if not self.CheckValid(x+dx,y+dy):
                no_collision=False
                break
            x+=dx
            y+=dy
            step+=1
        return (no_collision, x, y)
    
    # Check if near the target. If yes, add the target node.
    def IsNearTarget(self,q_new):
        dis=CalcNodeDist(q_new,self.q_target)
        if dis<=self.dis_goal:
            self.q_target.idx=len(self.q_list)
            self.q_target.idx_parent=q_new.idx
            self.AppendNode(self.q_target)
            self.PATH_FIND=1
            return True
        else:
            return False

    def AppendNode(self,q_new):
        self.q_list.append(q_new)

    def UpdateParent(self, q_new, near_inds):
        for ind in near_inds:
            node=self.q_list[ind]
            dis=CalcNodeDist(q_new, node)
            cost=node.cost+dis
            
            (no_collision, _, _)=self.WalkUntilCollision(node, q_new)
            if no_collision==False:
                continue
            
            if cost<q_new.cost:
                q_new.cost=cost
                q_new.idx_parent=node.idx
            
        return q_new
    
    def Rewire(self, q_new, near_inds):
        for ind in near_inds:
            node=self.q_list[ind]
            dis=CalcNodeDist(q_new, node)
            cost=q_new.cost+dis
            
            (no_collision, _, _)=self.WalkUntilCollision(node, q_new)
            if no_collision==False:
                continue
            
            if cost<node.cost:
                self.q_list[ind].cost=cost
                self.q_list[ind].idx_parent=q_new.idx
            
        return q_new        
    # ----------------------- PRINT ----------------------------#
    
    def print_nodes(self):
        l=self.q_list
        for i in range(len(l)):
            q=l[i]
            print q.x,q.y,q.idx,q.idx_parent

    # check if a point is valid: x>=self.size or x<0 or y>=self.size or y<0
    def CheckValid(self,x,y):
        x=int(x)
        y=int(y)
        if (x>=self.shape[1] or x<0 or y>=self.shape[0] or y<0):
            return False
        if self.img[y][x]==0:
            return False
        return True

    def ShowRRTTree(self):
        verts = []
        codes = []
        for i in range(1,len(self.q_list)):
            q_cur=self.q_list[i]
            self.push_node_for_plot(q_cur,verts,codes)
        path = Path(verts, codes)
        patch = patches.PathPatch(path,ec='y')
        ax = plt.gca()
        ax.add_patch(patch)

    def ShowPath(self): # show the path from the start to the end
        if self.PATH_FIND:
            verts = []
            codes = []
            q=self.q_list[-1]
            while q.idx!=0:
                self.push_node_for_plot(q,verts,codes)
                q=self.q_list[q.idx_parent]
            path = Path(verts, codes)
            patch = patches.PathPatch(path,ec='r')
            ax = plt.gca()
            ax.add_patch(patch)
        else:
            # print('path not found')
            None

    def show_RRT_result(self): # show the map, tree, and path
        mycv.imshow(self.img)
        self.ShowRRTTree()
        self.ShowPath()
        # show
        ax = plt.gca()
        ax.set_xlim([0,100])
        ax.set_ylim([0,100])

    def push_node_for_plot(self,q_cur,verts,codes):
        # push nodes' positions, in order to draw the lines between nodes
        q_pre=self.q_list[q_cur.idx_parent]
        verts.append([q_cur.x,q_cur.y])
        verts.append([q_pre.x,q_pre.y])
        codes.append(Path.MOVETO)
        codes.append(Path.LINETO)

if __name__ == '__main__':

    # add map
    if 1:
        img = mycv.imread(FILE_PATH+'/../images/map.png')
        img = mycv.rgb2bw(img)
    else:
        img =np.ones((100, 100))
        
    rand_ranges=[ [0, 30, 0, 100], [70, 100, 0, 100],
                 [0, 100, 0, 30], [0, 100, 70, 100]]
    for rand_range in rand_ranges:
        img=mycv.AddCircle(img, 
            num=2, radius=3, std=1, rand_range=rand_range)
  
    # init RRT
    q_start=Node(x=40,y=60)
    q_target=Node(x=60,y=40)

    rrt=RRT()
    rrt.init(img,q_start,q_target, expand_dis=5,
        dis_goal=8, goal_sample_rate=0.1, max_iter=2000)
    (find_target, steps)=rrt.Plan()

    # print result
    rrt.show_RRT_result()
    print 'steps =', steps,', nodes =',len(rrt.q_list)
    if rrt.PATH_FIND==0:
        print 'path not found'
    else:
        print 'path found !'
    
    mycv.savefig(IMAGE_PATH+"/RRTstar.png", border_size=10)
    plt.show()
