#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

typedef enum {
    MOTION_AXIS_PAN = 0,
    MOTION_AXIS_TILT = 1,
    MOTION_AXIS_COUNT = 2,
} motion_axis_id_t;

typedef struct {
    float pan_deg;
    float tilt_deg;
    float pan_speed_dps;
    float tilt_speed_dps;
    bool pan_homed;
    bool tilt_homed;
    bool outputs_enabled;
    bool homing;
    bool pan_min;
    bool pan_max;
    bool tilt_min;
    bool tilt_max;
} motion_status_t;

void motion_init(void);
void motion_set_enabled(bool enabled);
void motion_emergency_stop(bool active);
void motion_stop_all(void);
void motion_invalidate_home(void);

bool motion_set_manual_velocity(float pan_dps, float tilt_dps);
bool motion_goto(float pan_deg, float tilt_deg);
esp_err_t motion_home_all(void);
bool motion_wait_idle(uint32_t timeout_ms);

bool motion_is_homing(void);
bool motion_is_idle(void);
bool motion_all_homed(void);
void motion_get_status(motion_status_t *status);
