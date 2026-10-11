"""make small previews from the actual recorded simulation."""
from pathlib import Path
import cv2,imageio.v2 as imageio,numpy as np
from camera_tracker.simulation import Scene,ROOT
reports=ROOT/'reports'
reader=imageio.get_reader(str(reports/'nominal.mp4'))
frames=[cv2.resize(reader.get_data(i),(640,360)) for i in range(0,240,4)]
imageio.mimsave(str(reports/'tracking_preview.gif'),frames,duration=200,loop=0)
montage=np.concatenate([cv2.resize(reader.get_data(i),(800,450)) for i in (40,120,200)],axis=0)
cv2.imwrite(str(reports/'motion_frames.png'),cv2.cvtColor(montage,cv2.COLOR_RGB2BGR));reader.close()
scene=Scene();scene.set_angles(18,12)
cv2.imwrite(str(ROOT/'simulation/camera_mount_preview.png'),scene.overview(1280,720));scene.close()
print('rendered video previews and the current camera/light mount')
