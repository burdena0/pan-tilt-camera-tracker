#pragma once

#include <stdbool.h>

void controls_init(void);
void controls_calibrate_joystick(void);
void controls_read_joystick(float *pan, float *tilt);

bool controls_estop_open(void);
bool controls_track_selected(void);
bool controls_stop_pressed(void);
bool controls_home_pressed(void);
