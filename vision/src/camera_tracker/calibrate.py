import argparse
import hashlib
from pathlib import Path
import cv2
import numpy as np
from .geometry import Calibration

def board():
    dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    return cv2.aruco.CharucoBoard((7,5),.025,.018,dictionary)

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--images',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--board',action='store_true')
    args=parser.parse_args()
    target=board()
    if args.board:
        cv2.imwrite(str(args.output),target.generateImage((1400,1000),marginSize=40,borderBits=1));return
    detector=cv2.aruco.CharucoDetector(target)
    object_points,image_points=[],[];seen=set();size=None;coverage=np.zeros((3,4),bool)
    for path in sorted(args.images.glob('*')):
        if path.suffix.lower() not in ('.png','.jpg','.jpeg'):continue
        digest=hashlib.sha256(path.read_bytes()).hexdigest()
        if digest in seen:raise ValueError('duplicate calibration image')
        seen.add(digest)
        image=cv2.imread(str(path))
        if image is None:raise ValueError(f'unreadable image: {path}')
        current=(image.shape[1],image.shape[0])
        if size is not None and current!=size:raise ValueError('mixed calibration resolutions')
        size=current
        corners,ids,_,_=detector.detectBoard(image)
        if ids is None or len(ids)<12:continue
        objects,pixels=target.matchImagePoints(corners,ids)
        object_points.append(objects);image_points.append(pixels)
        for x,y in pixels.reshape(-1,2):coverage[min(2,int(y*3/size[1])),min(3,int(x*4/size[0]))]=True
    if len(object_points)<12 or coverage.sum()<8:raise ValueError('need 12 diverse usable views covering the image')
    rms,K,dist,rvecs,tvecs=cv2.calibrateCamera(object_points,image_points,size,None,None)
    errors=[]
    for objects,pixels,r,t in zip(object_points,image_points,rvecs,tvecs):
        projected,_=cv2.projectPoints(objects,r,t,K,dist)
        errors.append(float(np.sqrt(np.mean(np.sum((pixels.reshape(-1,2)-projected.reshape(-1,2))**2,axis=1)))))
    if rms>.8 or max(errors)>1.5:raise ValueError(f'calibration quality failed: rms={rms:.3f}, worst view={max(errors):.3f}')
    Calibration(K,dist,size,'measured',float(rms)).save(args.output)
    print(f'saved measured calibration: {len(object_points)} views, rms {rms:.3f}px')
if __name__=='__main__':main()
