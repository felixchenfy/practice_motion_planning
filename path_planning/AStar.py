#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
@author: feiyu
A* path planning
Ihe code is based the Pseudocode on Wiki

Core concept:
gScore[current]: distance from start to current
fScore = gScore[current] + self.heuristic_cost_estimate(current, q_goal)

"""

import numpy as np
import matplotlib.pyplot as plt
import math
import copy
import time

import os, sys
FILE_PATH=os.path.join(os.path.dirname(__file__))
IMAGE_PATH=FILE_PATH+'/../images'
sys.path.append(FILE_PATH + "/../lib_mylib")
from mycv import mycv
from mylib import CalcNodeDist, print_node, sigmoid, calc_norm

IF_ANIMATION=False
ANIMATION_GAP=100
IMG_SIZE=200

class Node(object):
    def __init__(self, x, y):
        self.x=int(x)
        self.y=int(y)
    # https://stackoverflow.com/questions/16306844/custom-comparison-functions-for-built-in-types-in-python
    def __eq__(self, other):
        return (self.x == other.x) and (self.y==other.y)
    def __hash__(self):
        return hash(self.x+self.y*IMG_SIZE)

class Astar(object):
    def __init__(self):
        return
    def Plan(self, img, q_start, q_goal, max_iter=10000):
        self.shape=img.shape
        self.img=img

        # The set of nodes already evaluated
        closedSet=set()
    
        # The set of currently discovered nodes that are not evaluated yet.
        # Initially, only the q_start node is known.
        openSet = {q_start}
    
        # For each node, which node it can most efficiently be reached from.
        # If a node can be reached from many nodes, dict_cameFrom will eventually contain the
        # most efficient previous step.
        dict_cameFrom = dict() # an empty std::unordered_map
    
        # For each node, the cost of getting from the q_start node to that node.
        gScore = dict() # with default value of Infinity
        
        # The cost of going from q_start to q_start is zero.
        gScore[q_start] = 0
    
        # For each node, the total cost of getting from the q_start node to the q_goal
        # by passing by that node. That value is partly known, partly heuristic.
        fScore = dict() # with default value of Infinity
    
        # For the first node, that value is completely heuristic.
        fScore[q_start] = self.heuristic_cost_estimate(q_start, q_goal)
    
        def get_fscore(q):
            return fScore[q]
        def set_img_show_color(node, fScore):
            y=node.y
            x=node.x
            val=int(fScore[q_current])
            colors=[0, val, 255-val]
            self.img_show[y,x]=colors
        
        iter=0
        find_target=False
        res_path=None

        while len(openSet) != 0: # while open set is not empty

            # q_current := the node in openSet having the lowest fScore[] value
            q_current = min(openSet, key=get_fscore)
            # see details about: min(iterable, *iterables[, key, default]):
            #       at https://www.programiz.com/python-programming/methods/built-in/min


            ## -------------------   Print and plot --------------------------##
            iter+=1
            if iter%100==0:
                print "iter=%d, len(openSet)=%d，len(closedSet)=%d"\
                    % (iter, len(openSet), len(closedSet))
            if iter==max_iter:
                break
            if iter==1: # init img_show
                self.img_show=mycv.gray2rgb(img)
                set_img_show_color(q_current, fScore)
            if IF_ANIMATION:
                if iter%ANIMATION_GAP==0: # draw
                    plt.clf()
                    mycv.imshow(self.img_show)
                    plt.pause(0.0001)
            ## ----------------------------------------------------------------##


            if self.is_same_node(q_current, q_goal):
                find_target=True
                res_path = self.reconstruct_path(dict_cameFrom, q_current)
                break 

            openSet.remove(q_current)
            closedSet.add(q_current)
    
            for neighbor in self.find_neighbors(q_current): # for each neighbor of q_current
               
                if neighbor in closedSet:
                    continue		# Ignore the neighbor which is already evaluated.
                
                # The distance from q_start to a neighbor
                tentative_gScore = gScore[q_current] + CalcNodeDist(q_current, neighbor)
    
                if neighbor not in openSet:	# Discover a new node
                    openSet.add(neighbor)
                elif tentative_gScore >= gScore[neighbor]:
                    continue		# This is not a better path.
    
                # This path is the best until now. Record it!
                dict_cameFrom[neighbor] = q_current
                gScore[neighbor] = tentative_gScore
                fScore[neighbor] = gScore[neighbor] + self.heuristic_cost_estimate(neighbor, q_goal)
                set_img_show_color(neighbor, fScore)
        
        
        self.find_target=find_target
        self.path=res_path
        return (self.find_target, self.path)

    def is_same_node(self, q1, q2):
        if q1.x==q2.x and q1.y==q2.y:
            return True
        else:
            return False

    def find_neighbors(self, q_current):
        x=q_current.x
        y=q_current.y
        neighbors_pos=[[x-1,y],[x+1,y],[x,y-1],[x,y+1]]
        neighbors=list()
        for p in neighbors_pos:
            q=Node(p[0], p[1])
            if not self.isValidNode(q):
                continue
            neighbors.append(q)
        return neighbors
        
    def isValidNode(self, q):
        if q.x>=0 and q.x<=self.shape[1]:
            if q.y>=0 and q.y<=self.shape[0]:
                if self.img[q.y][q.x]!=0:
                    return True
        return False
            
    def reconstruct_path(self, dict_cameFrom, q_current):
        # dict_cameFrom: dict = {child: parent, }
        total_path = [q_current]
        while q_current in dict_cameFrom:
            q_current = dict_cameFrom[q_current]
            total_path.append(q_current)
        return total_path
    
    # the total cost of getting from the q_start node to the q_goal
    def heuristic_cost_estimate(self, q_start, q_goal):    
        fScore = CalcNodeDist(q_start, q_goal)
        return fScore
            
if __name__ == '__main__':

    # add map
    if 1:
        img = mycv.imread(IMAGE_PATH+'/map.png')
        img = mycv.rgb2bw(img)
    else:
        img =np.zeros((100, 100))+255
        
    rand_ranges=[ [0, 30, 0, 100], [70, 100, 0, 100],
                    [0, 100, 0, 30], [0, 100, 70, 100]]
    for rand_range in rand_ranges:
        img=mycv.AddCircle(img, 
            num=2, radius=3, std=1, rand_range=rand_range)
  
    # Astar
    q_start=Node(x=40,y=60)
    q_target=Node(x=60,y=40)
    astar=Astar()
    (find_target, path)=astar.Plan(img,q_start,q_target)

    # print result
    if find_target:
        print "Find path"
    else:      
        print "Path not found"
    
    mycv.imshow(astar.img_show)
    if find_target:
        xs = [ node.x for node in astar.path]
        ys = [ node.y for node in astar.path]
        plt.plot(xs, ys, 'ro', ms=2)

    mycv.savefig(IMAGE_PATH+"/AStar.png")
    plt.show()
