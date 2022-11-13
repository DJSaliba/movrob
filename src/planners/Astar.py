from __future__ import annotations

from itertools import product

import numpy as np
from PIL import Image
import rospy

from utils.Node import Node, OpenList, VisitedList
from utils.utils import manh_dist
from planners.Planner import Planner


class Astar(Planner):
    def __init__(self,image_path, map_scale = 10,neighborhood=8):
        self.map_scale = map_scale

        self.image = Image.open(image_path).convert('1')
        self.size = np.array(self.image.size)
        self.scale = self.resolution/self.map_scale
        
        self.visited = VisitedList()
        self.openlist = OpenList()
        self.expanded = 0
        self.start.heuristic = self.heuristic(self.start.coord)
    
        if neighborhood not in [4,8]:
            raise ValueError
        
        self.neighborhood = neighborhood

    #TODO: find a better way to index, instead of tuple(zip(*List<Tuple>))
    def __call__(self,goal,position):
        coord = self.pos2coord(position)
        self.goal = self.pos2coord(goal)
        neightbors = self._get_neighbors(coord)
        self.start = Node(position,0,0,0,None,neightbors)

        rospy.loginfo("Calculating path")
        self.map = np.transpose(np.array(self.image)) - 1 # -1 is obstructed by obstacles

        node = self.search_loop()
        if not node:
            rospy.loginfo("No path available")
            return
        self.path = []
        while node != None:
            self.path.insert(0,node.coord)
            node = node.parent
        rospy.loginfo("Path calculated")
        return self

    def pos2coord(self,position):
        return (int(self.size[0]/2 + position[0]/self.scale),
                int(self.size[1]/2 - position[1]/self.scale))

    def _get_neighbors(self,position):
        x,y = position
        x = int(x)
        y = int(y)
        l = max(0, x-1)
        r = min(self.size[0],x+1)
        t = max(0, y-1)
        b = min(self.size[0],y+1)
        candidates = set(product([l,x,r],[b,y,t]))
        if self.neighborhood == 4:
            candidates = [c for c in candidates if manh_dist(c,(x,y))==1]
        return list(set([c for c in candidates if c != (x,y)]))

    def search_loop(self) -> Node|None:
        self.openlist.insert(self.start)
        while self.openlist:
            node = self.openlist.pop() # Best heuristic + cost
            if node.coord == self.goal:
                #print(self.expanded)
                return node
            self.visited.add(node.coord)
            child_nodes = self._get_neighbors(node.coord) # Generate childs
            child_nodes = [n for n in child_nodes if n not in self.visited]
            self.add2open(node,child_nodes) # Add to list handled in OpenList
        return None

    def heuristic(self, coord) -> float:
        return manh_dist(coord, self.goal)

    def add2open(self, parent:Node, children):
        self.expanded +=1
        for coord in children:
            if self.map[coord] == -1:
                self.visited.add(coord)
            node_cost = np.sqrt(manh_dist(parent.coord),coord)
            depth = parent.depth + 1
            heuristic = self.heuristic(coord)
            real_cost = node_cost+parent.real_cost
            neighbors = self._get_neighbors(coord)
            n = Node(coord,real_cost,heuristic,depth,parent,neighbors)
            self.openlist.insert(n)
