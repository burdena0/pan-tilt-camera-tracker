#include "turret_console.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "controls.h"
#include "motion.h"
#include "safety.h"
#include "vision_control.h"
static void handle_line(char *line)
{
    line[strcspn(line, "\r\n")] = 0;
    if (!line[0]) return;
    if (line[0] == '@') { vision_control_line(line); return; }
    float pan, tilt;
    int consumed = 0;
    if (!strcmp(line, "status")) vision_control_status();
    else if (!strcmp(line, "track on")) {
        printf("tracking %s\n", vision_control_enable() ? "enabled" : "rejected");
        vision_control_status();
    } else if (!strcmp(line, "track off") || !strcmp(line, "stop")) vision_control_disable();
    else if (!strcmp(line, "home") && !controls_track_selected() && !controls_estop_open() && !controls_stop_pressed() && !safety_fault_bits()) {
        vision_control_disable();
        printf("home result %d\n", motion_home_all());
    } else if (sscanf(line, "goto %f %f%n", &pan, &tilt, &consumed) == 2 &&
               line[consumed] == 0 && isfinite(pan) && isfinite(tilt) && !controls_estop_open() && !controls_stop_pressed() && !safety_fault_bits()) {
        vision_control_disable();
        if (motion_goto(pan, tilt)) motion_set_enabled(true);
    } else if (!strcmp(line, "clear")) printf("clear %s\n", safety_clear_faults() ? "accepted" : "rejected");
    else printf("commands: status, home, goto <pan> <tilt>, stop, track on|off, clear\n");
}
static void console_task(void *unused)
{
    (void)unused;
    char line[128];
    while (true) {
        if (fgets(line, sizeof(line), stdin)) {
            if (!strchr(line, '\n')) {
                int ch; while ((ch = getchar()) != '\n' && ch != EOF) {}
                vision_control_disable();
            } else handle_line(line);
        } else { clearerr(stdin); vTaskDelay(pdMS_TO_TICKS(20)); }
    }
}
void turret_console_start(void)
{
    BaseType_t created = xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, 4, NULL, 0);
    ESP_ERROR_CHECK(created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}
