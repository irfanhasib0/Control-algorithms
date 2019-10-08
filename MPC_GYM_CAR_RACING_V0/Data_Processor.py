# initial yaw compensation
'''
if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0
'''
#goal = [cx[-1], cy[-1]]


from collections import deque
import numpy as np
class Data_Processor():
    
    def __init__(self,_speed=1.0,tlen=15):
        self._stream=[]
        self.target_speed=_speed
        self.tlen=tlen
        self.no_of_st=4
        self.traj=np.zeros((self.no_of_st,self.tlen),dtype=np.float64)
    
    def update_trajectory(self,_tpoints):
        tx,ty=_tpoints[:,0],_tpoints[:,1]
        self.xs=np.array(tx)
        self.ys=np.array(ty)
        self.yaws=np.array(self.get_yaw())
        self.vs=np.array(self.get_speed_profile())
        
        self.traj[0,:]=self.xs
        self.traj[1,:]=self.ys
        self.traj[2,:]=self.vs
        self.traj[3,:]=self.yaws
        
        return self.traj
        
        
    def get_spline(self,tx,ty,dl):
        cx=tx
        cy=ty
        #cx, cy, _, _, _ = cubic_spline_planner.calc_spline_course(tx,ty,ds=dl)
        return tx,ty
    
    def get_yaw(self):
        
        dcx=np.gradient(self.xs)
        dcy=np.gradient(self.ys)
        cyaw=[np.arctan2(y,x) for x,y in zip(dcx,dcy)]
        return cyaw
        #sp = calc_speed_profile(cx, cy, cyaw, TARGET_SPEED)

    def get_speed_profile(self):

        speed_profile = np.ones(len(self.xs),dtype=np.float32)
        speed_profile*=self.target_speed
        speed_profile[-1] = 0.0

        return speed_profile

      
    def get_ref_trajectory(self,state,_dl,_dt,t_step):
        
        traj_points=self.traj.copy()
        xref = np.zeros((self.no_of_st, t_step + 1))
        dref = np.zeros((1, t_step + 1))

        xref[:, 0] = traj_points[:,0]
        dref[0, 0] = 0.0  # steer operational point should be 0

        travel = 0.0

        for i in range(t_step + 1):
            travel += abs(state.v) * _dt
            dind = int(round(travel / _dl))

            if (dind) < traj_points.shape[1]-1:
                xref[:, i] = traj_points[:,i]
                dref[0, i] = 0.0
            else:
                xref[:, i] = traj_points[:,-1]
                dref[0, i] = 0.0

        return xref,dref
