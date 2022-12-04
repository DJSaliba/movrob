from __future__ import annotations

from itertools import product

from tqdm import tqdm

import numpy as np
import random
from PIL import Image
import rospy
import matplotlib.pyplot as plt

from utils.Node import Node, OpenList, VisitedList
from utils.utils import vec_norm, manh_dist
from planners.Planner import Mapper


class QLearning(Mapper):
    def __init__(self,image_path, nit = 10000, scale = 1, neighborhood = 4, epsil = 0.1, alpha = 0.1, gama = 0.9):
        super().__init__(image_path)
        self.scale = scale
        self.nit = nit
        self.lin, self.col = self.size
        self.epsil = epsil
        self.alpha = alpha
        self.gama = gama

        if neighborhood not in [4,8]:
            raise ValueError
        
        self.neighborhood = neighborhood

    def _expand_obstacles(self) -> None:
        new_map = self.M.copy()
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if self.M[i,j] == -np.Inf:
                    continue
                else:
                    n = self._get_neighbors(i,j)
                    for neig in n:
                        if self.M[neig] == -np.Inf:
                            new_map[i,j] = -100
                            continue
        self.M = new_map

    def _get_neighbors(self,x,y,/,obj_hit=False) -> tuple:
        x = int(x)
        y = int(y)
        l = max(0, x-1)
        r = min(self.size[0],x+1)
        t = max(0, y-1)
        b = min(self.size[0],y+1)
        candidates = set(product([l,x,r],[b,y,t]))
        if obj_hit:
            candidates = [c for c in candidates if self.wave_map[c] >= 0]
        if self.neighborhood == 4:
            candidates = [c for c in candidates if abs(c[0]-x)+abs(c[1]-y)==1]
        return [c for c in candidates if c != (x,y)]

    def save_QL_img(self):
        img = plt.imread(self.image_path)
        plt.imshow(img)
        for i in tqdm(range(len(self.M))):
            for j in range(len(self.M[i])):
                if self.M[i][j] != -np.Inf and self.M[i][j] != 100:
                    ind_next = self.Q[i][j].index(max([e for e in self.Q[i][j] if isinstance(e, (float,int))]))
                    if ind_next == 0:
                        plt.plot(i,j, 'r', marker = (3, 0, 90), lw = 0.2, markersize = 5)
                    if ind_next == 1:
                        plt.plot(i,j, 'r', marker = (3, 0, 0), lw = 0.2, markersize = 5)
                    if ind_next == 2:
                        plt.plot(i,j, 'r', marker = (3, 0, 270), lw = 0.2, markersize = 5)
                    if ind_next == 3:
                        plt.plot(i,j, 'r', marker = (3, 0, 180), lw = 0.2, markersize = 5)
                if self.M[i][j] == 100:
                    plt.plot(i,j, 'ob', lw = 0.2, markersize = 5)
        for i in tqdm(range(len(self.path)-1)):
            plt.plot([self.path[i][0], self.path[i+1][0]], [self.path[i][1], self.path[i+1][1]], 'b-', linestyle = "-", lw = 1)
            
        plt.show()

    def __call__(self,goal,position):
        self._initialized = False
        self.path = []
        self.goal = self.pos2coord(goal)
        coord = self.pos2coord(position)
    
        self.Q = [[[0,0,0,0] for _ in range(self.col)] for _ in range(self.lin)]
        self.M = 1.0 * self.map
        self.M[np.where(self.M == 0)] = -np.Inf
        self.M[np.where(self.M == 1)] = -0.5 
        self._expand_obstacles()
        self.M[self.pos2coord(goal)] = 100
        rospy.loginfo("Calculating path")
        self._initialized = self.QLearning(coord)
        rospy.loginfo("Path calculated")

        return self
    
    def QLearning(self, coord):
        for epoch in tqdm(range(self.nit)):
            actual_pos = coord
            while True:
                x = actual_pos[0]
                y = actual_pos[1]
                if random.random() < self.epsil:
                    ind_next = self.Q[x][y].index(random.choice([i for i in self.Q[x][y] if isinstance(i, (float,int))]))
                else:
                    ind_next = self.Q[x][y].index(max([i for i in self.Q[x][y] if isinstance(i, (float,int))]))
                
                if ind_next == 0:
                    xnext = max(x - 1, 0)
                    ynext = y
                if ind_next == 1:
                    xnext = x
                    ynext = max(y - 1, 0)
                if ind_next == 2:
                    xnext = min(x + 1, self.col - 1)
                    ynext = y
                if ind_next == 3:
                    xnext = x
                    ynext = min(y + 1, self.lin - 1)
                self.Q[x][y][ind_next] = (1 - self.alpha) * self.Q[x][y][ind_next] + self.alpha * (self.M[xnext][ynext] + self.gama * max([i for i in self.Q[xnext][ynext] if isinstance(i, (float,int))]))
                actual_pos = [xnext, ynext]
                if self.M[xnext][ynext] == 100 or self.M[xnext][ynext] == -np.Inf:
                    break
        self.getPath(coord)
        self.save_QL_img()
        return True
                
    def verify_position(self,coord):
        current = self.map[coord]
        x,y = coord
        if x < 0 or y < 0 or x > self.size[0] or y > self.size[1]:
            msg = 'Out of bounds.'
            rospy.loginfo(msg)
            return False
        elif current == 0:
            msg = 'Inside a obstacle, please change position in world file'
            rospy.loginfo(msg)
            return False
        elif coord == self.goal:
            msg = 'Goal reached.'
            rospy.loginfo(msg)
            return False
        return True
    
    def getPath(self, coord):
        self.path.append(coord)
        newcoord = coord
        while newcoord != self.goal and self.M[newcoord[0]][newcoord[1]] != -np.Inf:
            ind_next = self.Q[newcoord[0]][newcoord[1]].index(max([e for e in self.Q[newcoord[0]][newcoord[1]] if isinstance(e, (float,int))]))
            if ind_next == 0:
                newcoord = (newcoord[0]-1, newcoord[1])
            if ind_next == 1:
                newcoord = (newcoord[0], newcoord[1]-1)
            if ind_next == 2:
                newcoord = (newcoord[0]+1, newcoord[1])
            if ind_next == 3:
                newcoord = (newcoord[0], newcoord[1]+1)
            if newcoord not in self.path:
                self.path.append(newcoord)
        
    def get_next(self, position):
        coord = self.pos2coord(position)
        x = int(np.floor(self.size[0]/2 + coord[0]/self.scale))
        y = int(np.floor(self.size[1]/2 - coord[1]/self.scale))

        if not self._initialized or not self.verify_position(coord):
            return None
        
        ind_next = self.Q[coord[0]][coord[1]].index(max([e for e in self.Q[coord[0]][coord[1]]if isinstance(e, (float,int))]))
        if ind_next == 0:
            return (-1,0)
        if ind_next == 1:
            return (0,1)
        if ind_next == 2:
            return (1,0)
        if ind_next == 3:
            return (0,-1)