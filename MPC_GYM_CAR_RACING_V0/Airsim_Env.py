#import gym
#from matplotlib import pyplot as plt
#import cv2
import numpy as np
#from gym_car_env import CarRacing
import time
#import cubic_spline_planner
import pickle
from Map_Tracker import Map_Tracker
import airsim
import math
#from AirSimClient import *
class State:
     
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.predelta = None
class Car_Env():
    
    def __init__(self,rec=False,qlen=5,max_len=50,path='',loop='1',seed=123):
        #self.ENV=CarRacing()
        self._path="city_map"#loop+'_map_as_'+str(seed)
        self.seed=seed
        self.max_len=max_len
        self.qlen=qlen
        self._save_path=False
        self._read_path=True
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
        self.track=track
    
    
    def save_path(self):
        f=open(self._path,'wb')
        pickle.dump(self.track,f)
        f.clse()
    
    def accl_to_gas(self,accl,vel):
        dt=1.0
        SIZE = 0.02
        ENGINE_POWER= 100000000*SIZE*SIZE
        WHEEL_MOMENT_OF_INERTIA = 4000*SIZE*SIZE
        WHEEL_R  = 27 
        wheel_rad=WHEEL_R*SIZE
        mass=(2*WHEEL_MOMENT_OF_INERTIA)/wheel_rade
        #mass=.25
        power=4*mass*accl*(2*vel+accl*dt)/2#
        #power=4*mass*accl*(vel+accl/2)
        gas=power/ENGINE_POWER
        return gas
    def get_image(self,cam,img_type='scene'):
        """
        Get image from AirSim client
        """
        if img_type=='scene':
            image_response = self.client.simGetImages([airsim.ImageRequest(cam, airsim.ImageType.Scene, False, False)])[0]
            image1d = np.fromstring(image_response.image_data_uint8, dtype=np.uint8)
            image_rgb = image1d.reshape(image_response.height, image_response.width, 3)
        if img_type=='dp':
            image_response = self.client.simGetImages([airsim.ImageRequest(cam, airsim.ImageType.DepthPlanner, True)])[0]
            image1d = np.array(image_response.image_data_float, dtype=np.float64)
            #image1d=np.array(image_response,dtype=np.float64)
            image_rgb = image1d.reshape(image_response.height, image_response.width)
        if img_type=='seg':
            image_response = self.client.simGetImages([airsim.ImageRequest(cam, airsim.ImageType.Segmentation, False, False)])[0]
            image1d = np.fromstring(image_response.image_data_uint8, dtype=np.uint8)
            image_rgb = image1d.reshape(image_response.height, image_response.width, 3)
        
        return image_rgb#[78:144,27:227,0:3]#.astype(np)
    
    
    def quaternion_to_euler(self,x, y, z, w):

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return [yaw, pitch, roll]
    
    def step(self,action,show_plot=False):

       
        
        #action[1]=self.accl_to_gas(action[1],self._state.v)
        #action[0]=action[0]-np.deg2rad(90)
        #action[0]*=-1
        
        self.steer=action[0]
        self.gas=action[1]
        self.brake=action[2]
        car_state = self.client.getCarState()
        
        if (car_state.speed <= self.gas):
            self.car_controls.throttle = 0.2
        else:
            self.car_controls.throttle = 0.0
        
        self.car_controls.throttle=self.gas
        self.car_controls.steering=self.steer
        self.car_controls.brake=self.brake
        
        
        self.client.setCarControls(self.car_controls)
        car_state = self.client.getCarState()
        pos=car_state.kinematics_estimated.position
        vel=car_state.speed
        angx=car_state.kinematics_estimated.orientation.x_val
        angy=car_state.kinematics_estimated.orientation.y_val
        angz=car_state.kinematics_estimated.orientation.z_val
        angw=car_state.kinematics_estimated.orientation.w_val
        
        [ang,_,_]=self.quaternion_to_euler(angx,angy,angz,angw)
        #a#ng*=-1
        #ang=ang+np.deg2rad(90)#-ang
        nst=[]
        nst.append(self.get_image('0'))
        #nst.append(self.get_image('1'))
        #nst.append(self.get_image('2'))
        #nst.append(self.get_image('3'))
        #nst.append(self.get_image('4'))
        
        nst.append(self.get_image('0',img_type='seg'))
        '''
        nst.append(self.get_image('1',img_type='seg'))
        nst.append(self.get_image('2',img_type='seg'))
        nst.append(self.get_image('3',img_type='seg'))
        nst.append(self.get_image('4',img_type='seg'))
        
        nst.append(self.get_image('0',img_type='dp'))
        nst.append(self.get_image('1',img_type='dp'))
        nst.append(self.get_image('2',img_type='dp'))
        nst.append(self.get_image('3',img_type='dp'))
        nst.append(self.get_image('4',img_type='dp'))
        '''
        rw=0
        ter=0
        info=0
        self._state= State(x=pos.x_val,y=pos.y_val,yaw=ang,v=vel) 
        _traj,_dind,_end=self._Map_Tracker.run(self._state)
        return [0,0,0,vel],[_traj,_dind,_end],[pos.x_val,pos.y_val,ang],nst,rw,ter,info

    

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
        self.client.reset()
        
        
        self.track=[]
        if self._save_path==True:
            self.save_path()
        if self._read_path ==True:
            self.read_path()
        
        
        #for i in range(25,50):
        #     points[i][1]=-5
        self.points=np.array(self.track,dtype=np.float64)
        self._Map_Tracker=Map_Tracker(self.points,qlen=self.qlen,rel_pos=True,max_len=self.max_len)
        #self.Dstr.get_planned_traj(self.points)
        
        #plt.plot(self.points[:,0],self.points[:,1])
        #plt.show()
        
    
       
        return [self.step([0,0,0]),self.points]
