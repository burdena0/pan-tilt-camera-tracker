#include "vision_control.h"
#include "vision_protocol.h"
#include "controls.h"
#include "motion.h"
#include "safety.h"
#include "esp_check.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
static SemaphoreHandle_t s_lock;
static vision_guard_t s_guard;
static uint32_t s_rejected;
static bool s_manual_active;
static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
static void disable_locked(void)
{
    vision_guard_disable(&s_guard);
    motion_stop_all();
    if (safety_fault_bits() || !motion_all_homed()) motion_set_enabled(false);
}
void vision_control_disable(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    disable_locked();
    xSemaphoreGive(s_lock);
}
bool vision_control_enable(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool permitted = safety_tracking_permitted() && motion_all_homed() && !motion_is_homing() && !s_manual_active;
    if (permitted) {
        motion_stop_all();
        uint32_t session = esp_random();
        vision_guard_enable(&s_guard, session ? session : 1, now_ms());
    }
    xSemaphoreGive(s_lock);
    return permitted;
}
bool vision_control_active(void)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool active = s_guard.enabled;
    xSemaphoreGive(s_lock);
    return active;
}
bool vision_control_line(const char *line)
{
    vision_packet_t packet;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    bool ok = vision_parse(line, &packet) && safety_tracking_permitted() &&
              motion_all_homed() && !motion_is_homing() &&
              vision_guard_accept(&s_guard, &packet, now_ms());
    if (ok) {
        ok = motion_set_manual_velocity(packet.pan_mdegps / 1000.0f, packet.tilt_mdegps / 1000.0f);
        if (ok) motion_set_enabled(true);
    }
    if (!ok) { ++s_rejected; disable_locked(); }
    xSemaphoreGive(s_lock);
    return ok;
}
void vision_control_status(void)
{
    motion_status_t status;
    motion_get_status(&status);
    char body[180];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    snprintf(body, sizeof(body), "@S1,%lu,%lu,%lu,%ld,%ld,%d,%d,%lu,%lu",
             (unsigned long)s_guard.session, (unsigned long)now_ms(),
             (unsigned long)s_guard.last_sequence, (long)(status.pan_deg * 1000),
             (long)(status.tilt_deg * 1000), status.pan_homed && status.tilt_homed,
             s_guard.enabled, (unsigned long)safety_fault_bits(), (unsigned long)s_rejected);
    xSemaphoreGive(s_lock);
    printf("\n%s*%04X\n", body, vision_crc(body, strlen(body)));
    fflush(stdout);
}
void vision_manual_override(float pan_dps, float tilt_dps)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    const bool active = pan_dps != 0.0f || tilt_dps != 0.0f;
    if (active) {
        vision_guard_disable(&s_guard);
        if (!safety_fault_bits() && !controls_estop_open() && !controls_stop_pressed() &&
            motion_set_manual_velocity(pan_dps, tilt_dps)) motion_set_enabled(true);
    } else if (s_manual_active) motion_set_manual_velocity(0.0f, 0.0f);
    s_manual_active = active;
    xSemaphoreGive(s_lock);
}
static void watchdog_task(void *unused)
{
    (void)unused;
    unsigned ticks = 0;
    while (true) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_guard.enabled && (vision_guard_expired(&s_guard, now_ms()) ||
            !safety_tracking_permitted() || !motion_all_homed() || motion_is_homing())) disable_locked();
        xSemaphoreGive(s_lock);
        if (++ticks == 2) { vision_control_status(); ticks = 0; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
void vision_control_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    ESP_ERROR_CHECK(s_lock ? ESP_OK : ESP_ERR_NO_MEM);
    BaseType_t created = xTaskCreatePinnedToCore(watchdog_task, "vision_watchdog", 4096, NULL, 9, NULL, 0);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
