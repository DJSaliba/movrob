from __future__ import annotations

from itertools import product

import numpy as np
from PIL import Image
import rospy

from utils.Node import Node, OpenList, VisitedList
from utils.utils import vec_norm, manh_dist
from planners.Planner import Planner


class Astar(Planner):
    def __init__(self,image_path, scale = 1,neighborhood=4):
        self.image = Image.open(image_path).convert('1')
        self.map = np.transpose(np.array(self.image))
        self.size = np.array(self.image.size)
        self.scale = scale
        
        self.visited = VisitedList()
        self.openlist = OpenList()
        self.expanded = 0
        self.path = []
        self._initialized = False

        if neighborhood not in [4,8]:
            raise ValueError
        
        self.neighborhood = neighborhood

    def gen_node(self,coord, add=False):
        neighbors = self._get_neighbors(coord,add)
        return Node(coord,0,0,0,None,neighbors)

    def __call__(self,goal,position):
        self._initialized = False
        
        rospy.loginfo("Calculating path")
        self.search_path(position,goal)
        rospy.loginfo("Path calculated")

        self._initialized = True
        return self

    def setup_search(self,position,goal):
        coord = self.pos2coord(position)
        self.goal = self.pos2coord(goal)
        start = self.gen_node(coord)
        self.openlist.insert(start)

    def search_path(self,position,goal):
        self.setup_search(position,goal)
        node = self.search_loop()
        if not node:
            rospy.loginfo("No path available")
            return
        self.path = []
        while node != None:
            self.path.insert(0,node.coord)
            node = node.parent

    def get_next(self, position, goal):
        if not self._initialized:
            return None
        coord = self.pos2coord(position,False)
        if not self.path:
            vec, norm = vec_norm(goal,position)
            if norm < 0.2:
                rospy.loginfo("Goal reached.")
                return None
        else:
            mid_goal = self.path[0]
            vec,norm =  vec_norm(coord,mid_goal)
            if norm < 0.2:
                self.path.pop(0)
                return (0,0)
        U = vec/norm
        if self.path:
            U[0] = -U[0]
        return U
    
    def pos2coord(self,position,exact = True):
        coord = ((self.size[0]-1)/2 + position[0]/self.scale,
                 (self.size[1]-1)/2 - position[1]/self.scale)
        if exact:
            return tuple(int(c) for c in coord)
        else:
            return coord

    def _get_neighbors(self,coord,add=False):
        x,y = coord
        x = int(x)
        y = int(y)
        l = max(0, x-1)
        r = min(self.size[0],x+1)
        t = max(0, y-1)
        b = min(self.size[0],y+1)
        candidates = set(product([l,x,r],[b,y,t]))
        if self.neighborhood == 4:
            candidates = [c for c in candidates if manh_dist(c,(x,y))==1]
        candidates = [c for c in candidates if self.map[c]]
        return list(set([c for c in candidates if c != (x,y)]))

    def search_loop(self) -> Node|None:
        while self.openlist:
            node = self.openlist.pop() # Best heuristic + cost
            if node.coord == self.goal:
                #print(self.expanded)
                return node
            self.visited.add(node.coord)
            child_nodes = node.neighbors # Get children and parent
            child_nodes = [n for n in child_nodes if n not in self.visited]
            self.add2open(node,child_nodes) # Add to list handled in OpenList
        return None

    def heuristic(self, coord) -> float:
        return vec_norm(coord, self.goal)[1]

    def add2open(self, parent:Node, children):
        self.expanded +=1
        for coord in children:
            if self.map[coord] == 0:
                self.visited.add(coord)
            node_cost = np.sqrt(manh_dist(parent.coord,coord))
            depth = parent.depth + 1
            heuristic = self.heuristic(coord)
            real_cost = node_cost+parent.real_cost
            neighbors = self._get_neighbors(coord)
            n = Node(coord,real_cost,heuristic,depth,parent,neighbors)
            self.openlist.insert(n)
