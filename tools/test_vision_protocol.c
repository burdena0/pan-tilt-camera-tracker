#include "vision_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static void frame(char *out, const char *body)
{
    sprintf(out, "%s*%04X", body, vision_crc(body, strlen(body)));
}
int main(void)
{
    assert(vision_crc("123456789",9)==0x29b1);
    vision_packet_t p;
    char line[128];
    frame(line,"@V1,123,1,1000,12000,-5000,250");
    assert(vision_parse(line,&p));
    assert(p.pan_mdegps==12000 && p.tilt_mdegps==-5000);
    vision_guard_t g;
    vision_guard_enable(&g,123,1000);
    assert(vision_guard_accept(&g,&p,1000));
    assert(!vision_guard_accept(&g,&p,1001));
    p.sequence=2;p.capture_ms=1001;p.session=124;
    assert(!vision_guard_accept(&g,&p,1002));
    p.session=123;p.pan_mdegps=45001;
    assert(!vision_guard_accept(&g,&p,1002));
    p.pan_mdegps=0;p.tilt_mdegps=-30001;
    assert(!vision_guard_accept(&g,&p,1002));
    p.tilt_mdegps=0;p.ttl_ms=301;
    assert(!vision_guard_accept(&g,&p,1002));
    p.ttl_ms=250;p.capture_ms=900;
    assert(!vision_guard_accept(&g,&p,1051));
    p.capture_ms=1020;
    assert(!vision_guard_accept(&g,&p,1000));
    p.capture_ms=1100;
    assert(vision_guard_accept(&g,&p,1100));
    assert(!vision_guard_expired(&g,1349));
    assert(vision_guard_expired(&g,1350));
    p.sequence=3;p.capture_ms=1350;
    assert(!vision_guard_accept(&g,&p,1350));
    vision_guard_disable(&g);
    assert(!vision_guard_accept(&g,&p,1351));
    vision_guard_enable(&g,123,0xfffffff0);
    p.sequence=0xffffffff;p.capture_ms=0xfffffff0;p.ttl_ms=100;
    assert(vision_guard_accept(&g,&p,0xfffffff5));
    p.sequence=0;p.capture_ms=20;
    assert(vision_guard_accept(&g,&p,21));
    assert(!vision_guard_expired(&g,119));
    assert(vision_guard_expired(&g,120));
    line[10]^=1;assert(!vision_parse(line,&p));
    const char *bad[]={"@V1,123,1,1000,NaN,0,250","@V1,4294967296,1,1000,0,0,250",
                      "@V1,123,1,1000,0,0,250,99","@V1,123,1,1000,0,0,250junk",
                      "@V1,123,1,1000,2147483648,0,250","@V1, 123,1,1000,0,0,250"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i){frame(line,bad[i]);assert(!vision_parse(line,&p));}
    puts("vision protocol: CRC, malformed frames, replay, stale data, bounds, watchdog and wraparound passed");
}
