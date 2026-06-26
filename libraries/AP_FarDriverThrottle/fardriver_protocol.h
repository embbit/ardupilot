#pragma once

#include <stdint.h>
#include <stdbool.h>

// Энумератор состояний конечного автомата для Си-файла
typedef enum {
    STATE_WAIT_MARKER,
    STATE_WAIT_ID,
    STATE_READ_DATA
} fardriver_parse_state_t;

typedef struct {
    uint16_t rpm;
    float voltage;
    float current;
    float line_current;   
    float phase_current;  
    uint8_t temperature_mot;
    uint8_t temperature_ecu;
    uint8_t charge_state;
} FardriverTelemetry;

#ifdef __cplusplus
extern "C" {
#endif

void fardriver_protocol_init(void);
void fardriver_create_power_packet(uint8_t power, uint8_t is_reverse, uint8_t *out_buffer);
bool fardriver_parse_telemetry(uint8_t byte, FardriverTelemetry *out_telemetry);

#ifdef __cplusplus
}
#endif
