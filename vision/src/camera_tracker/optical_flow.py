import cv2
import numpy as np

class TemporalProposals:
    """carry image features forward; the crop network still has to verify each box."""
    def __init__(self):self.gray=None;self.boxes=[]

    def propose(self,bgr):
        gray=cv2.GaussianBlur(cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY),(3,3),.7)
        result=[]
        if self.gray is not None and self.gray.shape==gray.shape:
            for box,label in self.boxes:
                mask=np.zeros(gray.shape,np.uint8)
                x1,y1,x2,y2=np.rint(box).astype(int)
                mask[max(0,y1):min(gray.shape[0],y2),max(0,x1):min(gray.shape[1],x2)]=255
                points=cv2.goodFeaturesToTrack(self.gray,30,.03,2,mask=mask,blockSize=3)
                if points is None or len(points)<4:continue
                moved,status,_=cv2.calcOpticalFlowPyrLK(self.gray,gray,points,None,winSize=(15,15),maxLevel=3)
                if moved is None:continue
                back,reverse,_=cv2.calcOpticalFlowPyrLK(gray,self.gray,moved,None,winSize=(15,15),maxLevel=3)
                if back is None:continue
                valid=(status.ravel()!=0)&(reverse.ravel()!=0)&(np.linalg.norm(back-points,axis=2).ravel()<.8)
                if valid.sum()<4:continue
                movement=(moved-points).reshape(-1,2)[valid]
                shift=np.median(movement,axis=0)
                inliers=np.linalg.norm(movement-shift,axis=1)<1.5
                if inliers.sum()<4 or inliers.mean()<.6 or np.linalg.norm(shift)>20:continue
                shifted=box+np.tile(np.median(movement[inliers],axis=0),2)
                if np.any(shifted[:2]<2) or np.any(shifted[2:]>[gray.shape[1]-2,gray.shape[0]-2]):continue
                result.append((shifted,label))
        self.next_gray=gray
        return result

    def commit(self,detections):
        self.gray=self.next_gray
        self.boxes=[(d.bbox.copy(),d.label) for d in detections]
