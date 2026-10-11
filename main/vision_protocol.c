#include "vision_protocol.h"
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>
uint16_t vision_crc(const char *data, size_t length)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; ++i) {
        crc ^= (uint16_t)(uint8_t)data[i] << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}
bool vision_parse(const char *line, vision_packet_t *packet)
{
    if (!line || !packet || strlen(line) > 120 || strncmp(line, "@V1,", 4)) return false;
    const char *star = strchr(line, '*');
    if (!star || strlen(star + 1) != 4) return false;
    for (int i = 1; i <= 4; ++i) {
        char c = star[i];
        if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))) return false;
    }
    if (vision_crc(line, (size_t)(star - line)) != strtoul(star + 1, NULL, 16)) return false;
    int64_t values[6];
    const char *cursor = line + 4;
    for (int i = 0; i < 6; ++i) {
        if (!((*cursor >= '0' && *cursor <= '9') || (*cursor == '-' && i >= 3 && i <= 4))) return false;
        errno = 0;
        char *end;
        long long value = strtoll(cursor, &end, 10);
        if (errno || end == cursor || end > star || (i < 5 ? *end != ',' : end != star)) return false;
        values[i] = value;
        cursor = end + 1;
    }
    for (int i = 0; i < 6; ++i) {
        if (i == 3 || i == 4) {
            if (values[i] < INT32_MIN || values[i] > INT32_MAX) return false;
        } else if (values[i] < 0 || values[i] > UINT32_MAX) return false;
    }
    *packet = (vision_packet_t){(uint32_t)values[0], (uint32_t)values[1], (uint32_t)values[2],
                              (int32_t)values[3], (int32_t)values[4], (uint32_t)values[5]};
    return true;
}
void vision_guard_enable(vision_guard_t *guard, uint32_t session, uint32_t now)
{
    *guard = (vision_guard_t){.session = session, .deadline_ms = now + VISION_MAX_TTL_MS,
                             .enabled = session != 0};
}
void vision_guard_disable(vision_guard_t *guard) { guard->enabled = false; }
bool vision_guard_expired(const vision_guard_t *guard, uint32_t now)
{
    return guard->enabled && (int32_t)(now - guard->deadline_ms) >= 0;
}
bool vision_guard_accept(vision_guard_t *guard, const vision_packet_t *packet, uint32_t now)
{
    if (!guard->enabled || vision_guard_expired(guard, now) || packet->session != guard->session) return false;
    const int32_t age = (int32_t)(now - packet->capture_ms);
    const int32_t advance = (int32_t)(packet->sequence - guard->last_sequence);
    if (age < -5 || age > (int32_t)VISION_MAX_AGE_MS ||
        packet->ttl_ms < 20 || packet->ttl_ms > VISION_MAX_TTL_MS ||
        age >= (int32_t)packet->ttl_ms || (guard->received && advance <= 0) ||
        packet->pan_mdegps < -45000 || packet->pan_mdegps > 45000 ||
        packet->tilt_mdegps < -30000 || packet->tilt_mdegps > 30000) return false;
    guard->last_sequence = packet->sequence;
    guard->received = true;
    guard->deadline_ms = packet->capture_ms + packet->ttl_ms;
    return true;
}
