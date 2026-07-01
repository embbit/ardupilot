#include "fardriver_protocol.h"

#define RX_MOTOR_PACKET_SIZE 27
#define CRC_HIGH_INDEX 25
#define CRC_LOW_INDEX 26

static fardriver_parse_state_t current_state = STATE_WAIT_MARKER;
static uint8_t rx_buffer[RX_MOTOR_PACKET_SIZE];
static uint8_t rx_index = 0;

void fardriver_protocol_init(void) 
{
    current_state = STATE_WAIT_MARKER;
    rx_index = 0;
}

void fardriver_create_power_packet(uint8_t power, uint8_t is_reverse, uint8_t *out_buffer) 
{
    out_buffer[0] = 0xAA;       
    out_buffer[1] = 0x00;       
    out_buffer[2] = power;      
    out_buffer[3] = is_reverse; 
    
    for (int i = 4; i < 14; i++) 
    {
        out_buffer[i] = 0x00;
    }

    uint16_t sum = 0;
    for (int i = 0; i < 14; i++) 
    {
        sum += out_buffer[i];
    }

    out_buffer[14] = (uint8_t)((sum >> 8) & 0xFF);
    out_buffer[15] = (uint8_t)(sum & 0xFF);
}

bool fardriver_parse_telemetry(uint8_t byte, FardriverTelemetry *out_telemetry) 
{
    switch (current_state) 
    {
        case STATE_WAIT_MARKER:
            if (byte == 0xAA) {
                rx_buffer[0] = byte;
                rx_index = 1;
                current_state = STATE_WAIT_ID;
            }
            break;

        case STATE_WAIT_ID:
            if (byte == 0x00) { // Отрезаем BMS, ждем пакет мотора
                rx_buffer[1] = byte;
                rx_index = 2;
                current_state = STATE_READ_DATA;
            } else {
                current_state = STATE_WAIT_MARKER;
                rx_index = 0;
            }
            break;

        case STATE_READ_DATA:
            if (rx_index < RX_MOTOR_PACKET_SIZE) {
                rx_buffer[rx_index] = byte;
                rx_index++;
            }

            if (rx_index == RX_MOTOR_PACKET_SIZE) {
                current_state = STATE_WAIT_MARKER;
                rx_index = 0;

                uint16_t calculated_crc = 0;
                for (int i = 0; i < CRC_HIGH_INDEX; i++) {
                    calculated_crc += rx_buffer[i];
                }

                uint16_t received_crc = (rx_buffer[CRC_HIGH_INDEX] << 8) | rx_buffer[CRC_LOW_INDEX];

                if (calculated_crc == received_crc) {
                    // Парсинг параметров строго по индексам байт ТЗ АНК
                    out_telemetry->rpm = (rx_buffer[2] << 8) | rx_buffer[3];

                    uint16_t raw_curr = (rx_buffer[4] << 8) | rx_buffer[5];
                    out_telemetry->current = (float)raw_curr / 4.0f;                
                    
                    uint16_t raw_volt = (rx_buffer[6] << 8) | rx_buffer[7];
                    out_telemetry->voltage = (float)raw_volt / 10.0f;
                    
                    out_telemetry->temperature_mot = rx_buffer[8];
                    out_telemetry->temperature_ecu = rx_buffer[9];
                    out_telemetry->charge_state    = rx_buffer[11];

                    uint16_t raw_line_curr = (rx_buffer[16] << 8) | rx_buffer[17];
                    out_telemetry->line_current = (float)raw_line_curr / 4.0f;

                    uint16_t raw_phase_curr = (rx_buffer[18] << 8) | rx_buffer[19];
                    out_telemetry->phase_current = (float)raw_phase_curr / 4.0f;

                    return true;
                }
            }
            break;
    }
    return false;
}
