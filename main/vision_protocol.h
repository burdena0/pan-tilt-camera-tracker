#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define VISION_MAX_AGE_MS 150U
#define VISION_MAX_TTL_MS 300U
typedef struct {
    uint32_t session, sequence, capture_ms;
    int32_t pan_mdegps, tilt_mdegps;
    uint32_t ttl_ms;
} vision_packet_t;
typedef struct {
    uint32_t session, last_sequence, deadline_ms;
    bool enabled, received;
} vision_guard_t;
uint16_t vision_crc(const char *data, size_t length);
bool vision_parse(const char *line, vision_packet_t *packet);
void vision_guard_enable(vision_guard_t *guard, uint32_t session, uint32_t now);
void vision_guard_disable(vision_guard_t *guard);
bool vision_guard_accept(vision_guard_t *guard, const vision_packet_t *packet, uint32_t now);
bool vision_guard_expired(const vision_guard_t *guard, uint32_t now);
