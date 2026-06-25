#include "AP_ModbusSteering.h"
#include <GCS_MAVLink/GCS.h>

extern "C" {
    void modbus_create_write_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t value, uint8_t *out_buffer);
    void modbus_create_write32_packet(uint8_t slave_id, uint16_t reg_addr, int32_t value, uint8_t *out_buffer);
    void modbus_create_read_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count, uint8_t *out_buffer);
    bool modbus_parse_read_response(uint8_t byte, uint8_t expected_bytes, StepperTelemetry *out_telemetry);
}

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_ModbusSteering::var_info[] = {
    AP_GROUPINFO("SLAVE_ID", 1, AP_ModbusSteering, slave_id, 1),
    AP_GROUPINFO("REG_ADDR", 2, AP_ModbusSteering, reg_address, 30),
    AP_GROUPINFO("MAX_STEPS", 3, AP_ModbusSteering, max_steps, 2000),
    AP_GROUPEND
};

AP_ModbusSteering::AP_ModbusSteering() {
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_ModbusSteering::init(AP_SerialManager &serial_manager) {
    _uart = serial_manager.find_serial((AP_SerialManager::SerialProtocol)101, 0);
    if (_uart != nullptr) {
        _uart->begin(115200); 
    }
}

void AP_ModbusSteering::update(float steering_out) {
    if (_uart == nullptr || !_uart->is_initialized()) {
        return; 
    }

    uint32_t now = AP_HAL::millis();
    uint32_t available_bytes = _uart->available();
    StepperTelemetry telemetry;
    const uint8_t expected_response_len = 7; // Length for function 0x03

    // FIX 1: Strictly limit the number of bytes read per tick.
    // On Mac M3, this protects the scheduler from freezing during packet parsing.
    if (available_bytes > 16) {
        available_bytes = 16;
    }

    for (uint32_t i = 0; i < available_bytes; i++) {
        uint8_t b = _uart->read();
        
        // Parse only answers to the read request (function 0x03)
        if (modbus_parse_read_response(b, expected_response_len, &telemetry)) {
            if (telemetry.error_code != 0) {
                // FIX: Limit the frequency of critical messages (once every 1000 ms)
                static uint32_t last_err_text_ms = 0;
                if (now - last_err_text_ms >= 1000) {
                    last_err_text_ms = now;
                    const char* err_msg = "Unknown error";
                    if (telemetry.error_code == 1) err_msg = "Overcurrent";
                    if (telemetry.error_code == 2) err_msg = "Bus overvoltage";
                    if (telemetry.error_code == 4) err_msg = "Encoder position error";
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "CL57R FAULT: %s", err_msg);
                }
            }
        }
    }

    // Check control packet transmission interval (20 Hz = every 50 ms)
    if ((now - _last_send_ms) < SEND_INTERVAL_MS) {
        return;
    }
    
    // FIX 2: To send the position packet (13b) + trigger (8b), we need 21b in the TX buffer.
    // If there is less space in TX, we exit to prevent locking the ArduPilot stream.
    if (_uart->txspace() < 22) {
        return;
    }
    _last_send_ms = now;

    static bool is_motor_enabled = false;
    static uint8_t query_toggle = 0; // 0 - control, 1 - error reading
    uint8_t tx_packet[16]; 

    // Initial enablement of winding power
    if (!is_motor_enabled) {
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0038, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        is_motor_enabled = true;
        
        gcs().send_text(MAV_SEVERITY_INFO, "CL57R: Steering motor coils are locked!");
        return; 
    }

    // FIX 3: New two-stroke state machine.
    // Eliminates the 150ms delay, making steering responsive.
    if (query_toggle == 0) {
        if (steering_out < -1.0f) steering_out = -1.0f;
        if (steering_out > 1.0f)  steering_out = 1.0f;

        // Step A: Calculate and immediately send target steps (13 bytes)
        int32_t target_pulses = (int32_t)(steering_out * max_steps.get());
        modbus_create_write32_packet((uint8_t)slave_id.get(), 0x0034, target_pulses, tx_packet);
        _uart->write(tx_packet, 13);

        // Step B: IMMEDIATELY send movement trigger (8 bytes), drive runs instantly
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0036, 0x0007, tx_packet);
        _uart->write(tx_packet, 8);

        query_toggle = 1; // Check drive status on the next tick
    } 
    else {
        // Query the drive error register (8 bytes)
        modbus_create_read_packet((uint8_t)slave_id.get(), 0x0004, 1, tx_packet);
        _uart->write(tx_packet, 8);
        
        query_toggle = 0; // Return to position control mode
    }
}