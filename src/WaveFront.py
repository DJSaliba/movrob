from PIL import Image
import numpy as np
from itertools import product



class WaveFront():
    def __init__(self,image_path,neighborhood=8):
        self.image = Image.open(image_path).convert('1')
        self.shape = self.image.size

        if neighborhood not in [4,8]:
            raise ValueError
        
        self.neighborhood = neighborhood
    
    def _expand_obstacles(self) -> None:
        new_map = self.wave_map.copy()
        for i in range(self.shape[0]):
            for j in range(self.shape[1]):
                if self.wave_map[i,j] == -1:
                    new_map[i,j] = -1
                else:
                    n = self._get_neighbors(i,j)
                    new_map[i,j] = min(self.wave_map[tuple(zip(*n))])

    def _get_neighbors(self,x:int,y:int,/,obj_hit=False) -> tuple:
        l = max(0, x-1)
        r = min(self.shape[0],x+1)
        t = max(0, y-1)
        b = min(self.shape[0],y+1)
        candidates = set(product([l,x,r],[b,y,t]))
        if obj_hit:
            candidates = [c for c in candidates if self.wave_map[c] != -1]
        if self.neighborhood == 4:
            candidates = [c for c in candidates if abs(c[0]-x)+abs(c[1]-y)==1]
        return [c for c in candidates if c != (x,y)]       

    #TODO: find a better way to index, instead of tuple(zip(*List<Tuple>))
    def __call__(self,goal):
        self.wave_map = np.array(self.image) -1 # -1 is obstructed by obstacles, 0 is free
        if self.neighborhood == 8:
            self._expand_obstacles()
            if self.wave_map[goal] != 0:
                raise RuntimeError
        self.goal = tuple(goal)

        # First iteration
        self.next_neighbors = self._get_neighbors(*goal,obj_hit=True)
        self.visited = {goal}
        self.wave_map[tuple(zip(*self.next_neighbors))] = 1
        while self.next_neighbors:
            self._wavefront()

        return self

    def get_next(self,x,y):
        current = self.wave_map[x,y]
        if current == 0 and (x,y) != self.goal or current == -1:
            raise RuntimeError
        candidates = self._get_neighbors(x,y,obj_hit=True)
        values = np.array([self.wave_map[c] for c in candidates])
        next_point = np.where(values == np.min(values))[0][0]
        next_direction = [x,y] - np.array(candidates[next_point])
        return next_direction

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
