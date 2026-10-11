#include "controls.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "turret_config.h"

static const char *TAG = "controls";
static adc_oneshot_unit_handle_t s_adc;
static int s_pan_center = 2048;
static int s_tilt_center = 2048;

static bool input_low(int pin)
{
    return gpio_get_level((gpio_num_t)pin) == 0;
}

void controls_init(void)
{
    const uint64_t input_mask =
        (1ULL << PIN_ESTOP_LOOP) |
        (1ULL << PIN_TRACK_SWITCH) |
        (1ULL << PIN_STOP_BUTTON) |
        (1ULL << PIN_HOME_BUTTON);

    gpio_config_t io = {
        .pin_bit_mask = input_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, JOYSTICK_PAN_ADC_CH, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, JOYSTICK_TILT_ADC_CH, &chan_cfg));
}

void controls_calibrate_joystick(void)
{
    int64_t pan_sum = 0;
    int64_t tilt_sum = 0;
    const int samples = 64;

    ESP_LOGI(TAG, "Leave joystick centered; calibrating...");
    for (int i = 0; i < samples; ++i) {
        int pan_raw = 0;
        int tilt_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc, JOYSTICK_PAN_ADC_CH, &pan_raw));
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc, JOYSTICK_TILT_ADC_CH, &tilt_raw));
        pan_sum += pan_raw;
        tilt_sum += tilt_raw;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    s_pan_center = (int)(pan_sum / samples);
    s_tilt_center = (int)(tilt_sum / samples);
    ESP_LOGI(TAG, "Joystick centers: pan=%d tilt=%d", s_pan_center, s_tilt_center);
}

static float normalize_axis(int raw, int center)
{
    int delta = raw - center;
    if (abs(delta) <= JOYSTICK_DEADBAND_RAW) {
        return 0.0f;
    }

    float span = delta > 0 ? (4095.0f - center) : (float)center;
    float magnitude = (fabsf((float)delta) - JOYSTICK_DEADBAND_RAW) /
                      (span - JOYSTICK_DEADBAND_RAW);
    magnitude = fminf(fmaxf(magnitude, 0.0f), 1.0f);
    /* square response gives fine control around center. */
    magnitude *= magnitude;
    return delta > 0 ? magnitude : -magnitude;
}

void controls_read_joystick(float *pan, float *tilt)
{
    int pan_raw = s_pan_center;
    int tilt_raw = s_tilt_center;
    if (adc_oneshot_read(s_adc, JOYSTICK_PAN_ADC_CH, &pan_raw) != ESP_OK ||
        adc_oneshot_read(s_adc, JOYSTICK_TILT_ADC_CH, &tilt_raw) != ESP_OK) {
        *pan = 0.0f;
        *tilt = 0.0f;
        return;
    }
    *pan = normalize_axis(pan_raw, s_pan_center);
    *tilt = -normalize_axis(tilt_raw, s_tilt_center);
}

/* the nc e-stop sense loop is healthy only while the input is held low. */
bool controls_estop_open(void)    { return !input_low(PIN_ESTOP_LOOP); }
bool controls_track_selected(void) { return input_low(PIN_TRACK_SWITCH); }
bool controls_stop_pressed(void) { return input_low(PIN_STOP_BUTTON); }
bool controls_home_pressed(void) { return input_low(PIN_HOME_BUTTON); }
