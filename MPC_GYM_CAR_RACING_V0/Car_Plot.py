import math
import numpy as np
def Draw_Car(cx,cy):
    dx=20#2
    dy=10#1
    pts=[[cx-dx,cy+dy],
    [cx+dx,cy+dy],
    [cx+dx,cy-dy],
    [cx-dx,cy-dy],
    [cx-dx,cy+dy]]
    pts=np.array(pts,dtype=np.float64)
    return pts
class Move_Contour():
    def __init__(self,points,XX,YY,phi):
        self.points=points.copy()
        self.XX=XX
        self.YY=YY
        self.ox=0
        self.oy=0
        self.rot_mat=np.array([[math.cos(phi),math.sin(phi)],[-math.sin(phi),math.cos(phi)]])
    def move(self):
        self.points[:,0]+=self.XX
        self.points[:,1]+=self.YY
    def get_origin(self):
        ox=0
        oy=0
        oo=np.mean(self.points[:-1],axis=0)
        self.ox=oo[0]
        self.oy=oo[1]
        #print(self.ox,self.oy)
        return self.ox,self.oy
    def shift_origin(self):
        self.points[:,0]-=np.float64(self.ox)
        self.points[:,1]-=np.float64(self.oy)
        return self.points
    def reverse_shift_origin(self):
        self.points[:,0]+=self.ox
        self.points[:,1]+=self.oy
        return self.points
    def rotate(self):
        for i in range(len(self.points)):
            self.points[i]=self.points[i].dot(self.rot_mat)
        return self.points
    def apply(self):
        self.move()
        self.get_origin()
        self.shift_origin()
        self.rotate()
        self.reverse_shift_origin()
        return self.points