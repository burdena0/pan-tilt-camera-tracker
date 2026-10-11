#include "motion.h"

#include <math.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "turret_config.h"
#include "controls.h"

typedef enum {
    COMMAND_VELOCITY,
    COMMAND_POSITION,
} command_mode_t;

typedef struct {
    gpio_num_t step_pin;
    gpio_num_t dir_pin;
    gpio_num_t min_pin;
    gpio_num_t max_pin;
    float steps_per_degree;
    float min_deg;
    float max_deg;
    float max_speed_dps;
    float accel_dps2;
    int32_t min_steps;
    int32_t max_steps;
    bool direction_inverted;

    volatile int32_t isr_rate_sps;
    /* configured travel is far below int32 range; 32-bit access is atomic. */
    volatile int32_t position_steps;
    volatile uint32_t phase;
    volatile bool pulse_high;
    volatile int8_t last_direction;
    volatile bool homed;

    command_mode_t command_mode;
    float command_velocity_dps;
    float command_position_deg;
    float current_velocity_dps;
} axis_t;

static const char *TAG = "motion";
static gptimer_handle_t s_step_timer;
static volatile bool s_outputs_requested;
static volatile bool s_outputs_active;
static volatile bool s_estop_active;
static volatile bool s_homing;

static axis_t s_axes[MOTION_AXIS_COUNT] = {
    [MOTION_AXIS_PAN] = {
        .step_pin = PIN_PAN_STEP,
        .dir_pin = PIN_PAN_DIR,
        .min_pin = PIN_PAN_MIN_LIMIT,
        .max_pin = PIN_PAN_MAX_LIMIT,
        .steps_per_degree = PAN_STEPS_PER_DEGREE,
        .min_deg = PAN_MIN_DEG,
        .max_deg = PAN_MAX_DEG,
        .max_speed_dps = PAN_MAX_SPEED_DPS,
        .accel_dps2 = PAN_ACCEL_DPS2,
        .direction_inverted = PAN_DIRECTION_INVERTED,
    },
    [MOTION_AXIS_TILT] = {
        .step_pin = PIN_TILT_STEP,
        .dir_pin = PIN_TILT_DIR,
        .min_pin = PIN_TILT_MIN_LIMIT,
        .max_pin = PIN_TILT_MAX_LIMIT,
        .steps_per_degree = TILT_STEPS_PER_DEGREE,
        .min_deg = TILT_MIN_DEG,
        .max_deg = TILT_MAX_DEG,
        .max_speed_dps = TILT_MAX_SPEED_DPS,
        .accel_dps2 = TILT_ACCEL_DPS2,
        .direction_inverted = TILT_DIRECTION_INVERTED,
    },
};

static float clampf(float value, float low, float high)
{
    return fminf(fmaxf(value, low), high);
}

static bool limit_active(gpio_num_t pin)
{
    /* nc limit loop opens at the stop, allowing the pull-up to read high. */
    return gpio_get_level(pin) != 0;
}

static int32_t axis_position_steps(axis_t *axis)
{
    return axis->position_steps;
}

static float axis_position_deg(axis_t *axis)
{
    return (float)axis_position_steps(axis) / axis->steps_per_degree;
}

static void axis_set_position_deg(axis_t *axis, float degrees)
{
    axis->position_steps = (int32_t)lroundf(degrees * axis->steps_per_degree);
    axis->phase = 0;
}

static void apply_driver_enable(void)
{
    const bool enable = s_outputs_requested && !s_estop_active;
    s_outputs_active = enable;
    gpio_set_level(PIN_DRIVERS_ENABLE, enable ? 0 : 1);
    if (!enable) {
        for (int i = 0; i < MOTION_AXIS_COUNT; ++i) {
            s_axes[i].isr_rate_sps = 0;
            s_axes[i].current_velocity_dps = 0.0f;
            gpio_set_level(s_axes[i].step_pin, 0);
            s_axes[i].pulse_high = false;
        }
    }
}

static bool IRAM_ATTR step_timer_alarm_cb(
    gptimer_handle_t timer,
    const gptimer_alarm_event_data_t *event_data,
    void *user_context)
{
    (void)timer;
    (void)event_data;
    (void)user_context;

    for (int i = 0; i < MOTION_AXIS_COUNT; ++i) {
        axis_t *axis = &s_axes[i];

        if (axis->pulse_high) {
            gpio_set_level(axis->step_pin, 0);
            axis->pulse_high = false;
        }

        const int32_t rate = axis->isr_rate_sps;
        if (!s_outputs_active || rate == 0) {
            axis->phase = 0;
            continue;
        }

        const int8_t direction = rate > 0 ? 1 : -1;
        if (axis->homed && ((direction < 0 && axis->position_steps <= axis->min_steps) ||
                            (direction > 0 && axis->position_steps >= axis->max_steps))) {
            axis->phase = 0;
            continue;
        }
        if ((direction < 0 && gpio_get_level(axis->min_pin) != 0) ||
            (direction > 0 && gpio_get_level(axis->max_pin) != 0)) {
            axis->phase = 0;
            continue;
        }

        if (axis->last_direction != direction) {
            const bool positive_level = axis->direction_inverted ? false : true;
            gpio_set_level(axis->dir_pin,
                           direction > 0 ? positive_level : !positive_level);
            axis->last_direction = direction;
            axis->phase = 0;
            continue; /* guarantees at least one isr period of dir setup time. */
        }

        axis->phase += (uint32_t)abs(rate);
        if (axis->phase >= STEP_ISR_HZ) {
            axis->phase -= STEP_ISR_HZ;
            gpio_set_level(axis->step_pin, 1);
            axis->pulse_high = true;
            axis->position_steps += direction;
        }
    }
    return false;
}

static float desired_axis_velocity(axis_t *axis)
{
    float desired = 0.0f;
    const float position = axis_position_deg(axis);

    if (axis->command_mode == COMMAND_POSITION && axis->homed) {
        const float error = axis->command_position_deg - position;
        if (fabsf(error) > (0.75f / axis->steps_per_degree)) {
            /* braking-limited profile: v <= sqrt(2*a*remaining distance). */
            const float braking_speed = sqrtf(2.0f * axis->accel_dps2 * fabsf(error));
            desired = copysignf(fminf(axis->max_speed_dps, braking_speed), error);
        }
    } else {
        desired = clampf(axis->command_velocity_dps,
                         -axis->max_speed_dps, axis->max_speed_dps);
    }

    if (axis->homed) {
        const float remaining = desired < 0 ? position-axis->min_deg : axis->max_deg-position;
        const float braking_speed = sqrtf(2.0f*axis->accel_dps2*fmaxf(0,remaining-1.0f/axis->steps_per_degree));
        desired = copysignf(fminf(fabsf(desired),braking_speed),desired);
        if ((position <= axis->min_deg && desired < 0.0f) ||
            (position >= axis->max_deg && desired > 0.0f) ||
            (limit_active(axis->min_pin) && desired < 0.0f) ||
            (limit_active(axis->max_pin) && desired > 0.0f)) {
            desired = 0.0f;
        }
    }
    return desired;
}

static void motion_planner_task(void *unused)
{
    (void)unused;
    const TickType_t period = pdMS_TO_TICKS(1000 / MOTION_PLANNER_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = 1.0f / MOTION_PLANNER_HZ;

    while (true) {
        for (int i = 0; i < MOTION_AXIS_COUNT; ++i) {
            axis_t *axis = &s_axes[i];
            const float desired = s_outputs_active ? desired_axis_velocity(axis) : 0.0f;
            const float max_change = axis->accel_dps2 * dt;
            const float delta = clampf(desired - axis->current_velocity_dps,
                                       -max_change, max_change);
            axis->current_velocity_dps += delta;

            if (fabsf(axis->current_velocity_dps) < 0.01f && desired == 0.0f) {
                axis->current_velocity_dps = 0.0f;
            }

            int32_t rate = (int32_t)lroundf(axis->current_velocity_dps *
                                            axis->steps_per_degree);
            const int32_t rate_limit = (STEP_ISR_HZ / 2) - 1;
            if (rate > rate_limit) rate = rate_limit;
            if (rate < -rate_limit) rate = -rate_limit;
            axis->isr_rate_sps = rate;
        }
        xTaskDelayUntil(&last_wake, period);
    }
}

void motion_init(void)
{
    for (int i = 0; i < MOTION_AXIS_COUNT; ++i) {
        s_axes[i].min_steps = (int32_t)lroundf(s_axes[i].min_deg*s_axes[i].steps_per_degree);
        s_axes[i].max_steps = (int32_t)lroundf(s_axes[i].max_deg*s_axes[i].steps_per_degree);
    }
    const uint64_t output_mask =
        (1ULL << PIN_PAN_STEP) |
        (1ULL << PIN_PAN_DIR) |
        (1ULL << PIN_TILT_STEP) |
        (1ULL << PIN_TILT_DIR) |
        (1ULL << PIN_DRIVERS_ENABLE);
    gpio_config_t outputs = {
        .pin_bit_mask = output_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&outputs));
    gpio_set_level(PIN_DRIVERS_ENABLE, 1);
    gpio_set_level(PIN_PAN_STEP, 0);
    gpio_set_level(PIN_TILT_STEP, 0);

    const uint64_t limit_mask =
        (1ULL << PIN_PAN_MIN_LIMIT) |
        (1ULL << PIN_PAN_MAX_LIMIT) |
        (1ULL << PIN_TILT_MIN_LIMIT) |
        (1ULL << PIN_TILT_MAX_LIMIT);
    gpio_config_t limits = {
        .pin_bit_mask = limit_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&limits));

    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &s_step_timer));

    gptimer_event_callbacks_t callbacks = {
        .on_alarm = step_timer_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(s_step_timer, &callbacks, NULL));

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = 1000000 / STEP_ISR_HZ,
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(s_step_timer, &alarm_cfg));
    ESP_ERROR_CHECK(gptimer_enable(s_step_timer));
    ESP_ERROR_CHECK(gptimer_start(s_step_timer));

    BaseType_t created = xTaskCreatePinnedToCore(
        motion_planner_task, "motion_planner", 4096, NULL, 8, NULL, 1);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_LOGI(TAG, "Motion initialized: %.3f pan and %.3f tilt steps/degree",
             PAN_STEPS_PER_DEGREE, TILT_STEPS_PER_DEGREE);
}

void motion_set_enabled(bool enabled)
{
    s_outputs_requested = enabled;
    apply_driver_enable();
}

void motion_emergency_stop(bool active)
{
    s_estop_active = active;
    if (active) {
        motion_stop_all();
    }
    apply_driver_enable();
}

void motion_stop_all(void)
{
    for (int i = 0; i < MOTION_AXIS_COUNT; ++i) {
        s_axes[i].command_mode = COMMAND_VELOCITY;
        s_axes[i].command_velocity_dps = 0.0f;
        if (s_estop_active) {
            s_axes[i].isr_rate_sps = 0;
            s_axes[i].current_velocity_dps = 0.0f;
        }
    }
}

bool motion_set_manual_velocity(float pan_dps, float tilt_dps)
{
    if (!isfinite(pan_dps) || !isfinite(tilt_dps)) return false;
    if (s_homing || !motion_all_homed() || s_estop_active) {
        return false;
    }
    s_axes[MOTION_AXIS_PAN].command_mode = COMMAND_VELOCITY;
    s_axes[MOTION_AXIS_TILT].command_mode = COMMAND_VELOCITY;
    s_axes[MOTION_AXIS_PAN].command_velocity_dps = pan_dps;
    s_axes[MOTION_AXIS_TILT].command_velocity_dps = tilt_dps;
    return true;
}

bool motion_goto(float pan_deg, float tilt_deg)
{
    if (!isfinite(pan_deg) || !isfinite(tilt_deg)) return false;
    if (s_homing || !motion_all_homed() || s_estop_active) {
        return false;
    }
    s_axes[MOTION_AXIS_PAN].command_position_deg =
        clampf(pan_deg, PAN_MIN_DEG, PAN_MAX_DEG);
    s_axes[MOTION_AXIS_TILT].command_position_deg =
        clampf(tilt_deg, TILT_MIN_DEG, TILT_MAX_DEG);
    s_axes[MOTION_AXIS_PAN].command_mode = COMMAND_POSITION;
    s_axes[MOTION_AXIS_TILT].command_mode = COMMAND_POSITION;
    return true;
}

static bool wait_for_limit(axis_t *axis, uint32_t timeout_ms)
{
    const TickType_t started = xTaskGetTickCount();
    while (!limit_active(axis->min_pin)) {
        if (s_estop_active || controls_estop_open() || controls_stop_pressed() || controls_track_selected() ||
            (xTaskGetTickCount() - started) > pdMS_TO_TICKS(timeout_ms)) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return true;
}

static bool home_axis(axis_t *axis, const char *name)
{
    ESP_LOGI(TAG, "Homing %s toward minimum switch", name);
    axis->homed = false;
    axis->command_mode = COMMAND_VELOCITY;
    axis->command_velocity_dps = -HOME_SPEED_DPS;

    if (!wait_for_limit(axis, HOME_AXIS_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "%s homing timed out or E-stop opened", name);
        axis->command_velocity_dps = 0.0f;
        return false;
    }

    axis->command_velocity_dps = 0.0f;
    vTaskDelay(pdMS_TO_TICKS(150));
    if (s_estop_active || controls_estop_open() || controls_stop_pressed() || controls_track_selected()) return false;
    axis_set_position_deg(axis, axis->min_deg);

    /* move off the switch so a stuck/broken nc loop is detected. */
    axis->command_velocity_dps = HOME_SPEED_DPS;
    const float release_target = axis->min_deg + HOME_BACKOFF_DEG;
    const TickType_t started = xTaskGetTickCount();
    while (axis_position_deg(axis) < release_target) {
        if (s_estop_active || controls_estop_open() || controls_stop_pressed() || controls_track_selected() ||
            (xTaskGetTickCount() - started) > pdMS_TO_TICKS(10000)) {
            axis->command_velocity_dps = 0.0f;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    axis->command_velocity_dps = 0.0f;
    vTaskDelay(pdMS_TO_TICKS(150));

    if (s_estop_active || controls_estop_open() || controls_stop_pressed() || controls_track_selected()) return false;

    if (limit_active(axis->min_pin)) {
        ESP_LOGE(TAG, "%s minimum switch did not release", name);
        return false;
    }
    axis->homed = true;
    ESP_LOGI(TAG, "%s homed at %.2f degrees", name, axis_position_deg(axis));
    return true;
}

esp_err_t motion_home_all(void)
{
    if (s_homing) return ESP_ERR_INVALID_STATE;
    if (s_estop_active || controls_estop_open() || controls_stop_pressed() || controls_track_selected()) return ESP_ERR_INVALID_STATE;

    s_homing = true;
    motion_stop_all();
    motion_set_enabled(true);

    bool ok = home_axis(&s_axes[MOTION_AXIS_PAN], "pan") &&
              home_axis(&s_axes[MOTION_AXIS_TILT], "tilt");
    motion_stop_all();
    s_homing = false;

    if (!ok) {
        s_axes[MOTION_AXIS_PAN].homed = false;
        s_axes[MOTION_AXIS_TILT].homed = false;
        motion_set_enabled(false);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

bool motion_wait_idle(uint32_t timeout_ms)
{
    const TickType_t started = xTaskGetTickCount();
    do {
        if (motion_is_idle()) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    } while ((xTaskGetTickCount() - started) <= pdMS_TO_TICKS(timeout_ms));
    return false;
}

bool motion_is_homing(void) { return s_homing; }

bool motion_is_idle(void)
{
    return abs(s_axes[MOTION_AXIS_PAN].isr_rate_sps) < 2 &&
           abs(s_axes[MOTION_AXIS_TILT].isr_rate_sps) < 2;
}

bool motion_all_homed(void)
{
    return s_axes[MOTION_AXIS_PAN].homed && s_axes[MOTION_AXIS_TILT].homed;
}
void motion_invalidate_home(void)
{
    motion_stop_all();
    s_axes[MOTION_AXIS_PAN].homed = false;
    s_axes[MOTION_AXIS_TILT].homed = false;
}

void motion_get_status(motion_status_t *status)
{
    status->pan_deg = axis_position_deg(&s_axes[MOTION_AXIS_PAN]);
    status->tilt_deg = axis_position_deg(&s_axes[MOTION_AXIS_TILT]);
    status->pan_speed_dps = s_axes[MOTION_AXIS_PAN].current_velocity_dps;
    status->tilt_speed_dps = s_axes[MOTION_AXIS_TILT].current_velocity_dps;
    status->pan_homed = s_axes[MOTION_AXIS_PAN].homed;
    status->tilt_homed = s_axes[MOTION_AXIS_TILT].homed;
    status->outputs_enabled = s_outputs_active;
    status->homing = s_homing;
    status->pan_min = limit_active(s_axes[MOTION_AXIS_PAN].min_pin);
    status->pan_max = limit_active(s_axes[MOTION_AXIS_PAN].max_pin);
    status->tilt_min = limit_active(s_axes[MOTION_AXIS_TILT].min_pin);
    status->tilt_max = limit_active(s_axes[MOTION_AXIS_TILT].max_pin);
}
