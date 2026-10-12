#pragma once

/*
 * Reference configuration for a future ESP32-DevKitC/WROOM-32 build
 * with two DRV8825 carriers at 1/16 microstepping. Hardware operation
 * is unverified; adapt these values to the assembled board and mechanics.
 */

/* step/direction outputs. */
#define PIN_PAN_STEP             25
#define PIN_PAN_DIR              26
#define PIN_TILT_STEP            27
#define PIN_TILT_DIR             14
#define PIN_DRIVERS_ENABLE       13  /* drv8825 nenbl, active low */

/* nc safety loops: closed = low/healthy, open = high/active or broken wire. */
#define PIN_PAN_MIN_LIMIT        16
#define PIN_PAN_MAX_LIMIT        17
#define PIN_TILT_MIN_LIMIT       18
#define PIN_TILT_MAX_LIMIT       19
#define PIN_ESTOP_LOOP           21

/* no controls: connect to ground when active; internal pull-ups are enabled. */
#define PIN_TRACK_SWITCH         22
#define PIN_STOP_BUTTON          23
#define PIN_HOME_BUTTON          32

/* analog joystick. gpio34/35 are adc1 inputs on the classic esp32. */
#define PIN_JOYSTICK_PAN         34
#define PIN_JOYSTICK_TILT        35
#define JOYSTICK_PAN_ADC_CH      ADC_CHANNEL_6
#define JOYSTICK_TILT_ADC_CH     ADC_CHANNEL_7
#define JOYSTICK_DEADBAND_RAW    260

/* original model: 200-step nema 17, 1/16 microsteps, 75:10 and 40:10 gears. */
#define MOTOR_FULL_STEPS_REV     200.0f
#define DRIVER_MICROSTEPS        16.0f
#define PAN_GEAR_RATIO           7.5f
#define TILT_GEAR_RATIO          4.0f

#define PAN_STEPS_PER_DEGREE \
    ((MOTOR_FULL_STEPS_REV * DRIVER_MICROSTEPS * PAN_GEAR_RATIO) / 360.0f)
#define TILT_STEPS_PER_DEGREE \
    ((MOTOR_FULL_STEPS_REV * DRIVER_MICROSTEPS * TILT_GEAR_RATIO) / 360.0f)

/* cable-safe travel. adjust only after measuring your installed hard stops. */
#define PAN_MIN_DEG              (-150.0f)
#define PAN_MAX_DEG              (150.0f)
#define TILT_MIN_DEG             (-25.0f)
#define TILT_MAX_DEG             (60.0f)

/* conservative starting motion values. measure the installed camera load. */
#define PAN_MAX_SPEED_DPS        45.0f
#define PAN_ACCEL_DPS2           80.0f
#define TILT_MAX_SPEED_DPS       30.0f
#define TILT_ACCEL_DPS2          55.0f
#define HOME_SPEED_DPS           8.0f
#define HOME_BACKOFF_DEG         5.0f
#define HOME_AXIS_TIMEOUT_MS     45000

/* step pulses come from a 20 khz gptimer isr. */
#define STEP_ISR_HZ              20000
#define MOTION_PLANNER_HZ        100

/* flip either value if positive commands move the physical axis backward. */
#define PAN_DIRECTION_INVERTED   0
#define TILT_DIRECTION_INVERTED  0

/* camera-only design assumptions. these are not measured hardware values. */
#define CAMERA_MASS_KG           0.15f
#define CAMERA_COG_OFFSET_M      0.025f
#define MOTOR_HOLDING_TORQUE_NM  0.59f
#define GEAR_EFFICIENCY          0.70f
#define MIN_STATIC_SAFETY_FACTOR 2.5f
