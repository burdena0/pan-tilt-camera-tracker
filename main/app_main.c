#include <stdbool.h>
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "controls.h"
#include "motion.h"
#include "safety.h"
#include "turret_config.h"
#include "turret_console.h"
#include "vision_control.h"
static void manual_control_task(void *unused)
{
    (void)unused;
    bool previous_home = false;
    TickType_t last_wake = xTaskGetTickCount();
    while (true) {
        bool home = controls_home_pressed();
        if (home && !previous_home && !controls_track_selected() &&
            !controls_estop_open() && !controls_stop_pressed() && !safety_fault_bits() && !motion_is_homing()) {
            vision_control_disable();
            motion_home_all();
            last_wake = xTaskGetTickCount();
        }
        previous_home = home;
        if (controls_stop_pressed()) vision_control_disable();
        else if (!motion_is_homing() && !safety_fault_bits()) {
            float pan, tilt;
            controls_read_joystick(&pan, &tilt);
            vision_manual_override(pan * PAN_MAX_SPEED_DPS, tilt * TILT_MAX_SPEED_DPS);
        }
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
void app_main(void)
{
    ESP_LOGI("tracker", "camera-only pan/tilt tracker starting");
    controls_init();
    motion_init();
    safety_start();
    controls_calibrate_joystick();
    vision_control_init();
    turret_console_start();
    BaseType_t created = xTaskCreatePinnedToCore(manual_control_task, "manual", 4096, NULL, 6, NULL, 0);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
