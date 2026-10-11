#pragma once
#include <stdbool.h>
void vision_control_init(void);
bool vision_control_enable(void);
void vision_control_disable(void);
bool vision_control_active(void);
bool vision_control_line(const char *line);
void vision_control_status(void);
void vision_manual_override(float pan_dps, float tilt_dps);
