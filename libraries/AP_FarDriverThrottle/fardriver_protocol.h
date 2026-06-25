#ifndef FARDRIVER_PROTOCOL_H
#define FARDRIVER_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t rpm;          
    float    voltage;      
    float    current;      
    int8_t   temperature;  
} FardriverTelemetry;

void fardriver_protocol_init(void);
void fardriver_create_power_packet(uint8_t power, uint8_t is_reverse, uint8_t *out_buffer);
bool fardriver_parse_telemetry(uint8_t byte, FardriverTelemetry *out_telemetry);

#ifdef __cplusplus
}
#endif

#endif //FARDRIVER_PROTOCOL_H
