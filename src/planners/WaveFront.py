import rospy
from PIL import Image
import numpy as np
from itertools import product



class WaveFront():
    def __init__(self, image_path, map_scale = 5,resolution = 1,neighborhood=4):
        self.map_scale = map_scale
        self.resolution = resolution

        self.image = Image.open(image_path).convert('1')
        self.image = self.image.resize(np.array(self.image.size)*self.map_scale)
        self.size = np.array(self.image.size)

        self.scale = self.resolution/self.map_scale

        if neighborhood not in [4,8]:
            raise ValueError
        
        self.neighborhood = neighborhood
    
    def _expand_obstacles(self) -> None:
        new_map = self.wave_map.copy()
        for i in range(self.size[0]):
            for j in range(self.size[1]):
                if self.wave_map[i,j] == -1:
                    new_map[i,j] = -1
                else:
                    n = self._get_neighbors(i,j)
                    new_map[i,j] = min(self.wave_map[tuple(zip(*n))])
        self.wave_map = new_map

    def _get_neighbors(self,x,y,/,obj_hit=False) -> tuple:
        x = int(x)
        y = int(y)
        l = max(0, x-1)
        r = min(self.size[0],x+1)
        t = max(0, y-1)
        b = min(self.size[0],y+1)
        candidates = set(product([l,x,r],[b,y,t]))
        if obj_hit:
            candidates = [c for c in candidates if self.wave_map[c] != -1]
        if self.neighborhood == 4:
            candidates = [c for c in candidates if abs(c[0]-x)+abs(c[1]-y)==1]
        return [c for c in candidates if c != (x,y)]       

    #TODO: find a better way to index, instead of tuple(zip(*List<Tuple>))
    def __call__(self,goal):
        rospy.loginfo("Calculating cost map")
        self.wave_map = np.transpose(np.array(self.image)) - 1 # -1 is obstructed by obstacles, 0 is free
        self.goal = (int(self.size[0]/2 + goal[0]/self.scale),
                     int(self.size[1]/2 - goal[1]/self.scale))
        #if self.neighborhood == 8:
        self._expand_obstacles()
        self._expand_obstacles()
        #    if self.wave_map[self.goal] != 0:
        #        raise RuntimeError
        self.real_goal = goal


        # First iteration
        self.next_neighbors = self._get_neighbors(*self.goal,obj_hit=True)
        self.visited = set(goal)
        if self.next_neighbors:
            self.wave_map[tuple(zip(*self.next_neighbors))] = 1
        while self.next_neighbors:
            self._wavefront()

        rospy.loginfo("Cost map complete")
        return self


    def get_next(self,x,y):
        x = int(np.floor(self.size[0]/2 + x/self.scale))
        y = int(np.floor(self.size[1]/2 - y/self.scale))
        if x<0 or y<0 or x>self.size[0] or y>self.size[1]:
            msg = 'Out of bounds.'
            rospy.loginfo(msg)
            return None

        current = self.wave_map[x,y]
        if current == 0 and (x,y) != self.goal:
            msg = 'No path to goal.'
            rospy.loginfo(msg)
            return None
        if current == -1:
            msg = 'Inside a obstacle, please change position in world file'
            rospy.loginfo(msg)
            return None
        if current <=(self.map_scale/5):
            msg = 'Goal reached.'
            rospy.loginfo(msg)
            return None
        
        candidates = self._get_neighbors(x,y,obj_hit=True)
        values = np.array([self.wave_map[c] for c in candidates])
        next_point = np.where(values == np.min(values))[0]
        candidates = [candidates[n] for n in next_point]
        values = np.array([abs(c[0]-self.goal[0])+abs(c[1]-self.goal[1]) for c in candidates])
        next_point = np.where(values == np.min(values))[0][0]
        next_direction = np.array(candidates[next_point]) - [x,y]
        next_direction[1] = -next_direction[1]
        return tuple(next_direction)

    def _wavefront(self):
        next = self.next_neighbors.pop(0)
        if next in self.visited:
            return
        else:
            self.visited.add(tuple(next))
        
        current = self.wave_map[next]
        neighbors = self._get_neighbors(*next,obj_hit=True)
        next_added = []
        for n in neighbors:
            if tuple(n) == self.goal:
                continue
            if self.wave_map[n] == 0:
                self.wave_map[n] = current + 1
                next_added.append(n)
        self.next_neighbors.extend(next_added)
