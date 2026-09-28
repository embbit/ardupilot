#include "modbus_protocol.h"

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (uint8_t bit = 0; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            } else {
                crc = (uint16_t)(crc >> 1);
            }
        }
    }
    return crc;
}

void modbus_create_write_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t value, uint8_t *out_buffer)
{
    out_buffer[0] = slave_id;
    out_buffer[1] = 0x06U; // FC06 write single register
    out_buffer[2] = (uint8_t)((reg_addr >> 8) & 0xFFU);
    out_buffer[3] = (uint8_t)(reg_addr & 0xFFU);
    out_buffer[4] = (uint8_t)((value >> 8) & 0xFFU);
    out_buffer[5] = (uint8_t)(value & 0xFFU);
    const uint16_t crc = modbus_crc16(out_buffer, 6);
    out_buffer[6] = (uint8_t)(crc & 0xFFU);
    out_buffer[7] = (uint8_t)((crc >> 8) & 0xFFU);
}

void modbus_create_write_multiple_packet(uint8_t slave_id, uint16_t start_reg, uint16_t reg_count,
                                         const uint16_t *reg_values, uint8_t *out_buffer)
{
    out_buffer[0] = slave_id;
    out_buffer[1] = 0x10U; // FC16 write multiple registers
    out_buffer[2] = (uint8_t)((start_reg >> 8) & 0xFFU);
    out_buffer[3] = (uint8_t)(start_reg & 0xFFU);
    out_buffer[4] = (uint8_t)((reg_count >> 8) & 0xFFU);
    out_buffer[5] = (uint8_t)(reg_count & 0xFFU);
    out_buffer[6] = (uint8_t)(reg_count * 2U);

    uint8_t idx = 7;
    for (uint16_t i = 0; i < reg_count; i++) {
        out_buffer[idx++] = (uint8_t)((reg_values[i] >> 8) & 0xFFU);
        out_buffer[idx++] = (uint8_t)(reg_values[i] & 0xFFU);
    }

    const uint16_t crc = modbus_crc16(out_buffer, idx);
    out_buffer[idx++] = (uint8_t)(crc & 0xFFU);
    out_buffer[idx] = (uint8_t)((crc >> 8) & 0xFFU);
}

void modbus_create_read_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count, uint8_t *out_buffer)
{
    out_buffer[0] = slave_id;
    out_buffer[1] = 0x03U; // FC03 read holding registers
    out_buffer[2] = (uint8_t)((reg_addr >> 8) & 0xFFU);
    out_buffer[3] = (uint8_t)(reg_addr & 0xFFU);
    out_buffer[4] = (uint8_t)((reg_count >> 8) & 0xFFU);
    out_buffer[5] = (uint8_t)(reg_count & 0xFFU);
    const uint16_t crc = modbus_crc16(out_buffer, 6);
    out_buffer[6] = (uint8_t)(crc & 0xFFU);
    out_buffer[7] = (uint8_t)((crc >> 8) & 0xFFU);
}
