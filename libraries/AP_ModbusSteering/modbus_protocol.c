#include "modbus_protocol.h"

typedef enum {
    MB_STATE_IDLE,
    MB_STATE_FUNCTION,
    MB_STATE_DATA,
    MB_STATE_CRC_L,
    MB_STATE_CRC_H
} mb_parse_state_t;

// Чтобы сделать код полностью потокобезопасным, эти переменные в будущем 
// лучше перенести внутрь структуры private-секции класса AP_ModbusSteering.h
static mb_parse_state_t parse_state = MB_STATE_IDLE;
static uint8_t rx_buf[32];
static uint8_t rx_idx = 0;
static uint8_t crc_low = 0;
static uint8_t expected_len_global = 0;

uint16_t modbus_crc16(const uint8_t *buf, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void modbus_create_write_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t value, uint8_t *out_buffer) {
    out_buffer[0] = slave_id;
    out_buffer[1] = 0x06;
    out_buffer[2] = (uint8_t)((reg_addr >> 8) & 0xFF);
    out_buffer[3] = (uint8_t)(reg_addr & 0xFF);
    out_buffer[4] = (uint8_t)((value >> 8) & 0xFF);
    out_buffer[5] = (uint8_t)(value & 0xFF);
    uint16_t crc = modbus_crc16(out_buffer, 6);
    out_buffer[6] = (uint8_t)(crc & 0xFF);
    out_buffer[7] = (uint8_t)((crc >> 8) & 0xFF);
}

void modbus_create_write_multiple_packet(uint8_t slave_id, uint16_t start_reg, uint16_t reg_count, const uint16_t *reg_values, uint8_t *out_buffer) {
    out_buffer[0] = slave_id;
    out_buffer[1] = 0x10; // Write Multiple Registers
    out_buffer[2] = (uint8_t)((start_reg >> 8) & 0xFF);
    out_buffer[3] = (uint8_t)(start_reg & 0xFF);
    out_buffer[4] = (uint8_t)((reg_count >> 8) & 0xFF);
    out_buffer[5] = (uint8_t)(reg_count & 0xFF);
    
    uint8_t byte_count = (uint8_t)(reg_count * 2);
    out_buffer[6] = byte_count;
    
    uint8_t idx = 7;
    for (uint16_t i = 0; i < reg_count; i++) {
        out_buffer[idx++] = (uint8_t)((reg_values[i] >> 8) & 0xFF);
        out_buffer[idx++] = (uint8_t)(reg_values[i] & 0xFF);
    }
    
    // Считаем CRC для всего пакета перед контрольной суммой (7 байт заголовка + байты данных)
    uint16_t crc = modbus_crc16(out_buffer, idx);
    out_buffer[idx++] = (uint8_t)(crc & 0xFF);
    out_buffer[idx++] = (uint8_t)((crc >> 8) & 0xFF);
}


void modbus_create_read_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count, uint8_t *out_buffer) {
    out_buffer[0] = slave_id;
    out_buffer[1] = 0x03; 
    out_buffer[2] = (uint8_t)((reg_addr >> 8) & 0xFF);
    out_buffer[3] = (uint8_t)(reg_addr & 0xFF);
    out_buffer[4] = (uint8_t)((reg_count >> 8) & 0xFF);
    out_buffer[5] = (uint8_t)(reg_count & 0xFF);
    uint16_t crc = modbus_crc16(out_buffer, 6);
    out_buffer[6] = (uint8_t)(crc & 0xFF);
    out_buffer[7] = (uint8_t)((crc >> 8) & 0xFF);
}

// КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: Добавлена жесткая валидация структуры Modbus RTU фрейма
bool modbus_parse_read_response(uint8_t byte, uint8_t expected_bytes, StepperTelemetry *out_telemetry) {
    if (expected_bytes > 32 || expected_bytes < 5) return false;

    switch (parse_state) {
        case MB_STATE_IDLE:
            if (byte >= 1 && byte <= 247) {
                rx_buf[0] = byte;
                rx_idx = 1;
                expected_len_global = expected_bytes;
                parse_state = MB_STATE_FUNCTION;
            }
            break;

        case MB_STATE_FUNCTION:
            rx_buf[rx_idx++] = byte;
            // ИСПРАВЛЕНИЕ: Проверяем код функции. Мы ждем только 0x03 (Read). 
            // Если пришел другой код или код ошибки (0x83), сбрасываем автомат, защищая планировщик ArduPilot.
            if (byte == 0x03) {
                parse_state = MB_STATE_DATA;
            } else {
                parse_state = MB_STATE_IDLE; // Сброс при мусоре
            }
            break;

        case MB_STATE_DATA:
            rx_buf[rx_idx++] = byte;
            if (rx_idx >= expected_len_global - 2) {
                parse_state = MB_STATE_CRC_L;
            }
            break;

        case MB_STATE_CRC_L:
            crc_low = byte;
            parse_state = MB_STATE_CRC_H;
            break;

        case MB_STATE_CRC_H: {
            uint16_t received_crc = (byte << 8) | crc_low;
            parse_state = MB_STATE_IDLE; 

            uint16_t calculated_crc = modbus_crc16(rx_buf, expected_len_global - 2);
            if (received_crc == calculated_crc) {
                uint32_t combined = ((uint32_t)rx_buf[3] << 24) |
                                    ((uint32_t)rx_buf[4] << 16) |
                                    ((uint32_t)rx_buf[5] << 8)  |
                                    (uint32_t)rx_buf[6];
                out_telemetry->actual_position = (int32_t)combined;
                out_telemetry->error_code = 0; 
                return true;
            }
            break;
        }
    }
    return false;
}