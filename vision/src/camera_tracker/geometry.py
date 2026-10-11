from dataclasses import dataclass
import json
from pathlib import Path
import cv2
import numpy as np


def camera_rotation(pan_deg, tilt_deg):
    pan, tilt = np.deg2rad([pan_deg, tilt_deg])
    cp, sp, ct, st = np.cos(pan), np.sin(pan), np.cos(tilt), np.sin(tilt)
    # base coordinates: forward, left, up. camera: right, down, forward.
    pan_R = np.array([[cp, -sp, 0], [sp, cp, 0], [0, 0, 1]])
    tilt_R = np.array([[ct, 0, -st], [0, 1, 0], [st, 0, ct]])
    return pan_R @ tilt_R @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])


@dataclass
class Calibration:
    K: np.ndarray
    distortion: np.ndarray
    size: tuple
    source: str = "synthetic"
    rms_px: float = 0

    def __post_init__(self):
        self.K = np.asarray(self.K, dtype=float)
        self.distortion = np.asarray(self.distortion, dtype=float).reshape(-1)
        if self.K.shape != (3, 3) or not np.isfinite(self.K).all() or not np.isfinite(self.distortion).all():
            raise ValueError("calibration contains invalid numbers")
        if self.K[0, 0] <= 0 or self.K[1, 1] <= 0 or tuple(self.K[2]) != (0, 0, 1):
            raise ValueError("invalid intrinsic matrix")
        if len(self.distortion) not in (4,5,8,12,14):raise ValueError('unsupported distortion coefficient count')
        if len(self.size) != 2 or min(self.size) <= 0 or self.source not in ("synthetic", "measured"):
            raise ValueError("invalid image size or calibration source")
        if not np.isfinite(self.rms_px) or self.rms_px < 0:
            raise ValueError("invalid reprojection error")

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        return cls(data['K'], data['distortion'], tuple(data['size']), data['source'], data['rms_px'])

    def save(self, path):
        Path(path).write_text(json.dumps(dict(K=self.K.tolist(), distortion=self.distortion.tolist(),
                                             size=self.size, source=self.source, rms_px=self.rms_px), indent=2))

    def bearing(self, pixel, pan, tilt):
        undistorted = cv2.undistortPoints(np.asarray(pixel, dtype=float).reshape(1, 1, 2), self.K, self.distortion)[0, 0]
        ray = camera_rotation(pan, tilt) @ np.r_[undistorted, 1]
        return np.rad2deg([np.arctan2(ray[1], ray[0]), np.arctan2(ray[2], np.hypot(ray[0], ray[1]))])

    def pixel(self, bearing, pan, tilt):
        azimuth, elevation = np.deg2rad(bearing)
        ray = np.array([np.cos(elevation)*np.cos(azimuth), np.cos(elevation)*np.sin(azimuth), np.sin(elevation)])
        local = camera_rotation(pan, tilt).T @ ray
        if local[2] <= 0: return None
        pixel, _ = cv2.projectPoints(local.reshape(1, 3), np.zeros(3), np.zeros(3), self.K, self.distortion)
        return pixel[0, 0]
