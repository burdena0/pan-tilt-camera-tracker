idf_component_register(
    SRCS
        "app_main.c"
        "controls.c"
        "motion.c"
        "vision_protocol.c"
        "vision_control.c"
        "safety.c"
        "turret_console.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_adc
        esp_driver_gpio
        esp_driver_gptimer
        esp_hw_support
        esp_timer
)
