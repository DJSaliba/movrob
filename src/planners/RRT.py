from __future__ import annotations

from itertools import product

import numpy as np
from numpy.random import uniform as rrange
from PIL import Image
import rospy

from utils.Node import Node, OpenList, VisitedList
from utils.utils import vec_norm, manh_dist, line_collision
from planners.Astar import Astar


class RRT(Astar):
    def __init__(self,image_path, scale = 1,neighborhood=4, n=3600):
        super().__init__(self,image_path, scale = 1,neighborhood=4)
        self.graph = {}
        self.gen_random_nodes(n)

    def gen_random_nodes(self,n):
        while len(self.graph) < n:
            x = rrange(0,self.size[0])
            y = rrange(0,self.size[1])
            coord = tuple(np.int64((x,y)))
            if self.map[coord]:
                self.graph[(x,y)] = self.gen_node((x,y))

    def _get_neighbors(self,coord,add=False):
        if coord in self.graph:
            return self.graph[coord].neighbors
        else:
            neighbors = []
            for node in self.graph:
                if self.check_collision(coord,node.coord):
                    continue
                neighbors.append(node.coord)
                if add:
                    self.graph[node.coord].neighbors.append(coord)
            return neighbors

    def check_collision(self,p1,p2):
        return line_collision(p1,p2,self.map)

    def setup_search(self, position, goal):
        coord = self.pos2coord(position)
        self.goal = self.pos2coord(goal)

        self.graph[coord] = self.gen_node(coord, add=True)
        self.graph[self.goal] = self.gen_node(self.goal, add=True)

        self.openlist.insert(self.graph[coord])
