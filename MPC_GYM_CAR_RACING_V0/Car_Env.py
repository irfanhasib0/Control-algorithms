import gym
from matplotlib import pyplot as plt
import cv2
import numpy as np
from gym_car_env import CarRacing
import time
#import cubic_spline_planner
import pickle
from Map_Tracker import Map_Tracker
class State:
     
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
class Car_Env():
    
    def __init__(self,rec=False,qlen=25,max_len=50):
        #self.ENV=CarRacing()
        self.max_len=max_len
        self.qlen=qlen
        self.ENV=gym.make('CarRacing-v0')
        if rec==True : self.ENV = gym.wrappers.Monitor(self.ENV, "Rec")
        self._save_path=False
        self._read_path=True
        self._state=State(x=0,y=0,yaw=0,v=0)
        self.points=[[0,0]]
        self.frame=np.zeros((100,200,3),dtype=np.uint8)
        

        #self.Dstr=Data_Streamer(15,TARGET_SPEED)
        
    def read_path(self):
        f=open('path-1','rb')
        track=pickle.load(f)
        f.close()
        self.ENV.track=track
    
    
    def save_path(self):
        f=open('path-2','wb')
        pickle.dump(self.ENV.track,f)
        f.close()
    
    def accl_to_gas(self,accl,vel):
        dt=1.0
        SIZE = 0.02
        ENGINE_POWER= 100000000*SIZE*SIZE
        WHEEL_MOMENT_OF_INERTIA = 4000*SIZE*SIZE
        WHEEL_R  = 27 
        wheel_rad=WHEEL_R*SIZE
        mass=(2*WHEEL_MOMENT_OF_INERTIA)/wheel_rad
        #mass=.25
        power=4*mass*accl*(2*vel+accl*dt)/2#
        #power=4*mass*accl*(vel+accl/2)
        gas=power/ENGINE_POWER
        return gas
    
    def step(self,action,show_plot=False):

       
        
        action[1]=self.accl_to_gas(action[1],self._state.v)
        #action[0]=action[0]-np.deg2rad(90)
        action[0]*=-1
        
        
        self.gas=action[1]
        self.steer=action[0]
        self.brake=action[2]
        
        nst,rw,ter,info=self.ENV.step(action) # take a random action
        self.frame[0:96,0:96,:]=np.array(nst)

        pos=self.ENV.car.hull.position
        vel=np.sqrt(self.ENV.car.hull.linearVelocity[0]**2+self.ENV.car.hull.linearVelocity[1]**2)
        ang=self.ENV.car.hull.angle
        #ang*=-1
        ang=ang+np.deg2rad(90)#-ang
        
            
        self._state= State(x=pos[0],y=pos[1],yaw=ang,v=vel) 
        #self.Dstr.get_origin_shift(self._state)
        #self.Dstr.shift_trajectory()
        _traj,_dind,_end=self._Map_Tracker.run(self._state)
        return [0,0,0,vel],[_traj,_dind,_end],[pos[0],pos[1],ang],nst,rw,ter,info

    

    def render(self):
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            plt.cla()
            img = cv2.putText(np.array(self.frame), 'vel %f'%vel, (100,10), font,.3, (255,255,0),1, cv2.LINE_AA)
            img = cv2.putText(np.array(img), 'ang %f'%ang, (100,20), font,.3, (255,0,255),1, cv2.LINE_AA)
            image = cv2.putText(np.array(img), 'x,y %f,%f'%(pos[0],pos[1]), (100,30), font,.2, (255,0,255),1, cv2.LINE_AA)

            plt.imshow(image)
            plt.show()
            plt.pause(0.0001)


    def reset(self):
        
        self.ENV.reset()
        
        if self._save_path==True:
            self.save_path()
        if self._read_path ==True:
            self.read_path()
        
        points=[[x,y] for a,b,x,y in self.ENV.track]
        self.points=np.array(points)
        self._Map_Tracker=Map_Tracker(self.points,qlen=self.qlen,rel_pos=True,max_len=self.max_len)
        #self.Dstr.get_planned_traj(self.points)
        
        plt.plot(self.points[:,0],self.points[:,1])
        plt.show()
        
    
       
        return [self.step([0,0,0]),self.points]
