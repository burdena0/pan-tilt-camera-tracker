import cv2
import numpy as np

def exposure_budget_ms(pixel_speed,blur_limit_px=1,minimum_ms=.2,maximum_ms=20):
    if not np.isfinite(pixel_speed) or pixel_speed<0:raise ValueError('pixel speed must be finite and nonnegative')
    return float(np.clip(1000*blur_limit_px/max(pixel_speed,1e-6),minimum_ms,maximum_ms))

def quality_metrics(bgr):
    gray=cv2.cvtColor(bgr,cv2.COLOR_BGR2GRAY)
    return {'mean_brightness':float(gray.mean()),'saturated_fraction':float(np.mean(gray>=250)),
            'laplacian_variance':float(cv2.Laplacian(gray,cv2.CV_64F).var())}
