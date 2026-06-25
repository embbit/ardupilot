#include "AP_FarDriverThrottle.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_FarDriverThrottle::var_info[] = {
    AP_GROUPINFO("MAX_RPM", 1, AP_FarDriverThrottle, max_rpm, 3000),
    AP_GROUPEND
};

AP_FarDriverThrottle::AP_FarDriverThrottle() {
    fardriver_protocol_init();
    AP_Param::setup_object_defaults(this, var_info);
}
void AP_FarDriverThrottle::init(AP_SerialManager &serial_manager) {
    _uart = serial_manager.find_serial((AP_SerialManager::SerialProtocol)100, 0);
    if (_uart != nullptr) {
        _uart->begin(57600); 
    }
}
void AP_FarDriverThrottle::update(float throttle_out) {
    if (_uart == nullptr || !_uart->is_initialized()) {
        return; 
    }

    uint32_t available_bytes = _uart->available();
    FardriverTelemetry telemetry_data;
    
    for (uint32_t i = 0; i < available_bytes; i++) {
        uint8_t b = _uart->read();
        if (fardriver_parse_telemetry(b, &telemetry_data)) {
            gcs().send_named_float("FD_Volt", telemetry_data.voltage);
            gcs().send_named_float("FD_Curr", telemetry_data.current);
            gcs().send_named_float("FD_Temp", (float)telemetry_data.temperature);
            gcs().send_named_float("FD_RPM",  (float)telemetry_data.rpm);

            gcs().send_text(MAV_SEVERITY_INFO, "FD: %u RPM | %.1fV | %.1fA | %dC", 
                            telemetry_data.rpm, 
                            (double)telemetry_data.voltage, 
                            (double)telemetry_data.current, 
                            telemetry_data.temperature);
        }
    }

    uint32_t now = AP_HAL::millis();
    if (now - _last_send_ms < SEND_INTERVAL_MS) {
        return;
    }
    if (_uart->txspace() < 16) {
        return; 
    }
    _last_send_ms = now;

    uint8_t target_power = 0;
    uint8_t target_dir = 0;

    if (throttle_out < 0.0f) {
        target_dir = 1; 
        target_power = (uint8_t)(-throttle_out * 100.0f);
    } else {
        target_dir = 0; 
        target_power = (uint8_t)(throttle_out * 100.0f);
    }
    if (target_power > 100) target_power = 100;

    static uint8_t CurrentPower = 0;
    static uint8_t CurrentDir = 0;
    static uint32_t ReverseDelayTimer = 0;

    if (CurrentDir != target_dir) {
        if (CurrentPower > 0) {
            CurrentPower = (CurrentPower > 5) ? (CurrentPower - 5) : 0;
        } else {
            if (ReverseDelayTimer == 0) ReverseDelayTimer = now;
            if (now - ReverseDelayTimer >= 2000) {
                CurrentDir = target_dir;
                ReverseDelayTimer = 0;
            }
        }
    } else {
        ReverseDelayTimer = 0;
        if (CurrentPower < target_power) {
            CurrentPower++;
        } else if (CurrentPower > target_power) {
            CurrentPower = (CurrentPower > 2) ? (CurrentPower - 2) : 0;
        }
    }

    uint8_t tx_packet[16];
    fardriver_create_power_packet(CurrentPower, CurrentDir, tx_packet);
    _uart->write(tx_packet, 16);
}
