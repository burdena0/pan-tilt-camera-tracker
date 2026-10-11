#pragma once

#include <stdbool.h>
#include <stdint.h>

void safety_start(void);
bool safety_tracking_permitted(void);
bool safety_clear_faults(void);
uint32_t safety_fault_bits(void);
const char *safety_fault_text(void);
