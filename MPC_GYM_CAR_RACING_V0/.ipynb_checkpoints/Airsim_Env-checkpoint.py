import gym
from matplotlib import pyplot as plt
import cv2
import numpy as np
from gym_car_env import CarRacing
import time
#import cubic_spline_planner
import pickle
from Map_Tracker import Map_Tracker
import airsim
#from AirSimClient import *
class State:
     
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
class Car_Env():
    
    def __init__(self,rec=False,qlen=5,max_len=50,seed=123):
        #self.ENV=CarRacing()
        self._path='map_as_'+str(seed)
        self.seed=seed
        self.max_len=max_len
        self.qlen=qlen
        self._save_path=True
        self._read_path=False
        self._state=State(x=0,y=0,yaw=0,v=0)
        self.points=[[0,0]]
       
        
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

        self.car_controls.steering = 0
        self.car_controls.throttle = 0
        self.car_controls.brake = 0

        
        
    def read_path(self):
        f=open(self._path,'rb')
        track=pickle.load(f)
        f.close()
        self.ENV.track=track
    
    
    def save_path(self):
        f=open(self._path,'wb')
        pickle.dump(self.track,f)
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

       
        
        #action[1]=self.accl_to_gas(action[1],self._state.v)
        #action[0]=action[0]-np.deg2rad(90)
        #action[0]*=-1
        
        self.steer=action[0]
        self.gas=action[1]
        self.brake=action[2]
        car_state = self.client.getCarState()
        
        if (car_state.speed < 2):
            self.car_controls.throttle = 1.0
        else:
            self.car_controls.throttle = 0.0
        
        #car_controls.throttle=self.gas
        self.car_controls.steering=self.steer
        self.car_controls.brake=self.brake
        
        
        self.client.setCarControls(self.car_controls)
        car_state = self.client.getCarState()
        pos=car_state.position
        vel=car_state.speed
        ang=car_state.orientation.z_val
        #ang*=-1
        #ang=ang+np.deg2rad(90)#-ang
        
            
        self._state= State(x=pos.x_val,y=pos.y_val,yaw=ang,v=vel) 
        _traj,_dind,_end=self._Map_Tracker.run(self._state)
        return [0,0,0,vel],[_traj,_dind,_end],[pos.x,pos.y,ang],nst,rw,ter,info

    

    def render(self,mode='rgb_array'):
            #img_raw=self.ENV.render(mode=mode)
            #font = cv2.FONT_HERSHEY_SIMPLEX
            #plt.cla()
            #img = cv2.putText(np.array(self.frame), 'vel %f'%vel, (100,10), font,.3, (255,255,0),1, cv2.LINE_AA)
            #img = cv2.putText(np.array(img), 'ang %f'%ang, (100,20), font,.3, (255,0,255),1, cv2.LINE_AA)
            #image = cv2.putText(np.array(img), 'x,y %f,%f'%(pos[0],pos[1]), (100,30), font,.2, (255,0,255),1, cv2.LINE_AA)

            #plt.imshow(image)
            #plt.show()
            #plt.pause(0.0001)
            return 0#img_raw

    def reset(self):
        
        self.track=[[0,0],[1,0],[2,0],[3,0],[4,0],[5,0],[6,0],[7,0],[8,0],[9,0],[10,0]]
        
        if self._save_path==True:
            self.save_path()
        if self._read_path ==True:
            self.read_path()
        
        points=[[x,y] for x,y in self.track]
        self.points=np.array(points)
        self._Map_Tracker=Map_Tracker(self.points,qlen=self.qlen,rel_pos=True,max_len=self.max_len)
        #self.Dstr.get_planned_traj(self.points)
        
        plt.plot(self.points[:,0],self.points[:,1])
        plt.show()
        
    
       
        return [self.step([0,0,0]),self.points]
