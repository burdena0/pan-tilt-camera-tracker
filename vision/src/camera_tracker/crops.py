import cv2
import numpy as np

def crop_rgb(bgr,bbox,size=64):
    x1,y1,x2,y2=map(float,bbox)
    side=max(x2-x1,y2-y1)*1.35
    cx,cy=(x1+x2)/2,(y1+y2)/2
    # same padding and resizing for training and inference.
    transform=np.array([[size/side,0,(side/2-cx)*size/side],
                        [0,size/side,(side/2-cy)*size/side]],np.float32)
    crop=cv2.warpAffine(bgr,transform,(size,size),borderMode=cv2.BORDER_REPLICATE)
    return cv2.cvtColor(crop,cv2.COLOR_BGR2RGB).transpose(2,0,1)
def normalize_contrast(crop):
    """smooth sensor noise, then stretch the crop's shared RGB range."""
    smooth=cv2.GaussianBlur(crop.transpose(1,2,0),(3,3),.7).astype(np.float32)
    low,high=np.percentile(smooth,[2,98])
    return np.clip((smooth-low)*255/max(high-low,12),0,255).astype(np.uint8).transpose(2,0,1)
