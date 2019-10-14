# initial yaw compensation
'''
if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0
'''
#goal = [cx[-1], cy[-1]]

import numpy as np
from collections import deque
class Map_Tracker():
    
    def __init__(self,_stream,qlen=25,rel_pos=False,max_len=50):
        self._stream=_stream
        self.rel_pos=rel_pos
        self.point_stream=deque(maxlen=qlen)
        self.qlen=qlen
        self.max_len=max_len
        self.index=0
        self.dind=0
        self.end=False
        self.x,self.y=self._stream[:self.max_len,0],self._stream[:self.max_len,1]
        for _ in range(self.qlen):
            ind=self.index
            self.point_stream.append([self.x[ind],self.y[ind]])
            self.index+=1
            
    def add_points(self,num):
        
        for _ in range(num):
            ind=self.index
            if ind<len(self.x)-1:
                self.point_stream.append([self.x[ind],self.y[ind]])
                self.index+=1
            else:
                self.end=True
                break
        return self.end
   
    def get_origin_shift(self,state):
        
        traj_points=self.point_stream.copy()
        d = [(state.x-idx) ** 2 + (state.y-idy) ** 2 for (idx, idy) in traj_points]
        mind = min(d)
        self.dind = d.index(mind)
        return self.dind
    
    def shift_trajectory(self):
        
        if self.dind:
            self.end=self.add_points(self.dind)
        return self.end 
    
    def run(self,state):
        self.dind=self.get_origin_shift(state)
        self.end=self.shift_trajectory()
        new_traj=np.array(self.point_stream.copy())
        if self.rel_pos==True:
            new_traj[:,0]-=state.x
            new_traj[:,1]-=state.y
        return new_traj,self.dind,self.end