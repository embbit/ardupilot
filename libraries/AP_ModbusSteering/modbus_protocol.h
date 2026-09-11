#ifndef MODBUS_PROTOCOL_H
#define MODBUS_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
    int16_t actual_position; 
    uint16_t error_code;     
} StepperTelemetry;

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
void modbus_create_write_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t value, uint8_t *out_buffer);
void modbus_create_write_multiple_packet(uint8_t slave_id, uint16_t start_reg, uint16_t reg_count, const uint16_t *reg_values, uint8_t *out_buffer);
void modbus_create_read_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count, uint8_t *out_buffer);
bool modbus_parse_read_response(uint8_t byte, uint8_t expected_bytes, StepperTelemetry *out_telemetry);

#ifdef __cplusplus
}
#endif

#endif
