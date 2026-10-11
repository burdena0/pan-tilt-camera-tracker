import numpy as np
import pytest
from camera_tracker.image_quality import exposure_budget_ms,quality_metrics

def test_exposure_budget_limits_motion_to_one_pixel():
    assert exposure_budget_ms(500)==2
    assert exposure_budget_ms(100)==10
    assert exposure_budget_ms(0)==20
    with pytest.raises(ValueError):exposure_budget_ms(np.nan)

def test_brightness_quality_detects_dark_and_clipped_images():
    dark=quality_metrics(np.zeros((32,32,3),np.uint8))
    bright=quality_metrics(np.full((32,32,3),255,np.uint8))
    assert dark['mean_brightness']==0 and dark['saturated_fraction']==0
    assert bright['mean_brightness']==255 and bright['saturated_fraction']==1
