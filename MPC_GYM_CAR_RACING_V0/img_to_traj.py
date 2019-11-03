import cv2
import numpy as np
class image_to_traj():
    def __init__(self,N=25):
        self.N=N
    #@property
    def run(self,image):
        canny = self.do_canny(image)
        segment = self.do_segment(canny)
        ret= self.get_points(segment)
        return ret
        #hough = cv2.HoughLinesP(segment, 1, np.pi / 180, 10, np.array([]), minLineLength = 5, maxLineGap = 5)
    def do_canny(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        canny = cv2.Canny(blur, 50, 150)
        return canny

    def do_segment(self,img):
        height = img.shape[0]
        polygons = np.array([
                                [(0, 80), (96, 80), (0, 0),(96,0)]
                            ])
        mask = np.zeros_like(img)
        cv2.fillPoly(mask, polygons, 255)
        segment = cv2.bitwise_and(img, mask)
        return segment

    def get_points(self,segment):
        crops=[]
        cxs=[]
        _cys=[]
        cys=[]
        offset=self.N//2
        height=segment.shape[1]
        for i in range(height//self.N):
            crop=segment[i*self.N:(i+1)*self.N,:]
            crops.append(crop)
            M=cv2.moments(crop)
            try:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
            except:
                pass
                #cx=20
                #cy=N
            cxs.append(cx)
            _cys.append(cy)
            cys.append(offset+cy)
            #print(cx,cy)
            #print(crop)
            _crop=crop.copy()
            img = cv2.circle(_crop,(cx,cy), 2, (255,255,255), -1)
            #plt.imshow(img,cmap='gray',vmin=0,vmax=255)
            #plt.show()_
            offset+=self.N
            #print(i)
        return crops,cxs,_cys,cys