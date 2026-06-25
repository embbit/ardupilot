#include "fardriver_protocol.h"

typedef enum {
    STATE_WAIT_MARKER,
    STATE_READ_DATA,
    STATE_READ_CRC_H,
    STATE_READ_CRC_L
} fardriver_parse_state_t;

static fardriver_parse_state_t current_state = STATE_WAIT_MARKER;
static uint8_t rx_buffer[16];
static uint8_t rx_index = 0;
static uint8_t crc_high = 0;

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
                current_state = STATE_READ_DATA;
            }
            break;

        case STATE_READ_DATA:
            rx_buffer[rx_index++] = byte;
            if (rx_index >= 14) {
                current_state = STATE_READ_CRC_H;
            }
            break;

        case STATE_READ_CRC_H:
            crc_high = byte;
            current_state = STATE_READ_CRC_L;
            break;

        case STATE_READ_CRC_L: {
            uint16_t received_crc = (crc_high << 8) | byte;
            current_state = STATE_WAIT_MARKER; 

            uint16_t calculated_crc = 0;
            for (int i = 0; i < 14; i++)
            {
                calculated_crc += rx_buffer[i];
            }

            if (received_crc == calculated_crc) 
            {
                out_telemetry->rpm = (rx_buffer[4] << 8) | rx_buffer[5];
                
                uint16_t raw_volt = (rx_buffer[6] << 8) | rx_buffer[7];
                out_telemetry->voltage = raw_volt / 10.0f;
                
                uint16_t raw_curr = (rx_buffer[8] << 8) | rx_buffer[9];
                out_telemetry->current = raw_curr / 10.0f;
                
                out_telemetry->temperature = (int8_t)rx_buffer[10];
                return true; 
            }
            break;
        }
    }
    return false;
}

