#include "safety.h"
#include "controls.h"
#include "motion.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
enum { FAULT_ESTOP = 1U, FAULT_LIMIT_CONFLICT = 2U };
static volatile uint32_t s_faults;
static volatile bool s_track_seen_off;
static bool limit_conflict(void)
{
    motion_status_t status;
    motion_get_status(&status);
    return (status.pan_min && status.pan_max) || (status.tilt_min && status.tilt_max);
}
static void safety_task(void *unused)
{
    (void)unused;
    while (true) {
        if (!controls_track_selected()) s_track_seen_off = true;
        if (controls_estop_open()) s_faults |= FAULT_ESTOP;
        if (limit_conflict()) s_faults |= FAULT_LIMIT_CONFLICT;
        if (controls_stop_pressed()) motion_stop_all();
        if (s_faults) {
            s_track_seen_off = false;
            motion_emergency_stop(true);
            motion_set_enabled(false);
            motion_invalidate_home();
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
void safety_start(void)
{
    BaseType_t created = xTaskCreatePinnedToCore(safety_task, "safety", 4096, NULL, 10, NULL, 1);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
bool safety_tracking_permitted(void)
{
    return !s_faults && !controls_estop_open() && s_track_seen_off && controls_track_selected() && !controls_stop_pressed();
}
bool safety_clear_faults(void)
{
    if (controls_estop_open() || controls_track_selected() || limit_conflict()) return false;
    s_faults = 0;
    s_track_seen_off = true;
    motion_emergency_stop(false);
    motion_set_enabled(false);
    return true;
}
uint32_t safety_fault_bits(void) { return s_faults; }
const char *safety_fault_text(void)
{
    switch (s_faults) {
        case 0: return "none";
        case FAULT_ESTOP: return "ESTOP";
        case FAULT_LIMIT_CONFLICT: return "LIMIT_CONFLICT";
        default: return "ESTOP+LIMIT_CONFLICT";
    }
}
