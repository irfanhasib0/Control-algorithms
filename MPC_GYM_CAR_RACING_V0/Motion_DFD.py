import numpy as np
import math
from copy import copy, deepcopy
class State:
     
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
        
    def array(self):
        return np.array([self.x,self.y,self.v,self.yaw],dtype=np.float64).copy()
    
class Motion():

    def __init__(self,DT=0.2,WB=5):
        pass
        
    
    @staticmethod
    def get_next_state_2(curr_state, v,w,DT=0.2):

        next_x = curr_state.x + curr_state.v * math.cos(curr_state.yaw) * DT
        next_y = curr_state.y + curr_state.v * math.sin(curr_state.yaw) * DT
        next_yaw = curr_state.yaw + w * DT
        next_v =   v# curr_state.v + a * DT
        #print(curr_state.v,a,DT,next_v)
        

        return State(x=next_x,y=next_y,v=next_v,yaw=next_yaw),[0,0,0]
    @staticmethod
    def get_next_state(x_state, v, w):
        #if curr_state==-1:
        #    curr_state=self.state
        
        curr_state=deepcopy(x_state)
        A, B, C = Motion.get_linear_model_matrix(v, curr_state.yaw, w)
        
        X=np.array([curr_state.x,curr_state.y,curr_state.yaw],dtype=np.float64)
        U=np.array([v,w],dtype=np.float64)
        [next_x,next_y,next_yaw]=np.matmul(A,X) + np.matmul(B,U)+ C
        next_v=v
        
        
        return State(x=next_x,y=next_y,v=next_v,yaw=next_yaw),[A,B,C]
    
    @staticmethod
    def get_trajectory(_init_state,accels, deltas,time_step):
        xbar =np.zeros((NX, time_step + 1))
        xbar[:, 0] = self.state.x,self.state.y,self.state.v,self.state.yaw
        #_state=State(x=state.x,y=state.y,v=state.v,yaw=state.yaw)
        _state=deepcopy(_init_state)
        for (accel, delta, i) in zip(accels, deltas, range(1, time_step + 1)):
            next_state = Motion.get_next_state(self,accel, delta, curr_state=_state)
            xbar[:, i] = next_state.array()
            _state=deepcopy(next_state)
        

        return xbar


    @staticmethod
    def get_linear_model_matrix(v, phi, delta,NX=3,NU=2,DT=0.2,WB=5):
                A = np.zeros((NX, NX))
                A[0, 0] = 1.0
                A[1, 1] = 1.0
                A[2, 2] = 1.0
                #A[3, 3] = 1.0
                #A[0, 2] = DT * math.cos(phi)
                A[0, 2] = - DT * v * math.sin(phi)#
                #A[1, 2] = DT * math.sin(phi)
                A[1, 2] = DT * v * math.cos(phi)#
                #A[3, 2] = DT * math.tan(delta) / WB

                B = np.zeros((NX, NU))
                B[0, 0] = math.cos(phi)*DT
                B[1, 0] = math.sin(phi)*DT
                B[2, 1] = DT

                C = np.zeros(NX)
                C[0] = DT * v * math.sin(phi) * phi
                C[1] = - DT * v * math.cos(phi) * phi
                #C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)
                #C = np.zeros(NX)
                return A, B, C
