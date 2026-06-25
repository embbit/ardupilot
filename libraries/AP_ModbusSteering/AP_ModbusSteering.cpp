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

    uint32_t available_bytes = _uart->available();
    StepperTelemetry telemetry;
    const uint8_t expected_response_len = 7;

    for (uint32_t i = 0; i < available_bytes; i++) {
        uint8_t b = _uart->read();
        if (modbus_parse_read_response(b, expected_response_len, &telemetry)) {
            if (telemetry.error_code != 0) {
                const char* err_msg = "Неизвестная ошибка";
                if (telemetry.error_code == 1) err_msg = "Перегрузка по току";
                if (telemetry.error_code == 2) err_msg = "Перенапряжение шины";
                if (telemetry.error_code == 4) err_msg = "Рассогласование энкодера";
                gcs().send_text(MAV_SEVERITY_CRITICAL, "CL57R АВАРИЯ: %s", err_msg);
            }
        }
    }

    uint32_t now = AP_HAL::millis();
    if ((now - _last_send_ms) < SEND_INTERVAL_MS) {
        return;
    }
    if (_uart->txspace() < 13) {
        return;
    }
    _last_send_ms = now;

    static bool is_motor_enabled = false;
    static uint8_t step_state = 0;
    uint8_t tx_packet[16]; 

    if (!is_motor_enabled) {
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0038, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        is_motor_enabled = true;
        gcs().send_text(MAV_SEVERITY_INFO, "CL57R: Обмотки привода руля заблокированы!");
        return; 
    }

    if (step_state == 0) {
        if (steering_out < -1.0f) steering_out = -1.0f;
        if (steering_out > 1.0f)  steering_out = 1.0f;

        int32_t target_pulses = (int32_t)(steering_out * max_steps.get());
        modbus_create_write32_packet((uint8_t)slave_id.get(), 0x0034, target_pulses, tx_packet);
        _uart->write(tx_packet, 13);
        step_state = 1;
    } 
    else if (step_state == 1) {
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0036, 0x0007, tx_packet);
        _uart->write(tx_packet, 8);
        step_state = 2;
    }
    else {
        modbus_create_read_packet((uint8_t)slave_id.get(), 0x0004, 1, tx_packet);
        _uart->write(tx_packet, 8);
        step_state = 0;
    }
}
