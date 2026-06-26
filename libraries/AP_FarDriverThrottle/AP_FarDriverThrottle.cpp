#include "AP_FarDriverThrottle.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

// Анонимное пространство имен для изоляции констант внутри этого файла
namespace {
    // Настройки портов и связи
    constexpr AP_SerialManager::SerialProtocol SERIAL_PROTO_FARDRIVER = (AP_SerialManager::SerialProtocol)100;
    constexpr uint32_t DEFAULT_BAUDRATE = 57600;

    // Лимиты буферов передачи/приема
    // КРИТИЧЕСКОЕ ИСПРАВЛЕНИЕ: Буфер увеличен до 64, так как входящий фрейм АНК стал 27 байт
    constexpr uint32_t MAX_RX_BYTES_PER_LOOP = 64; 
    constexpr uint32_t MIN_TX_SPACE_REQUIRED = 16;
    constexpr uint32_t PACKET_SIZE_BYTES = 16;

    // Временные интервалы (мс)
    // ИСПРАВЛЕНИЕ: Изменено на 500 мс согласно ТЗ вывода телеметрии
    constexpr uint32_t TELEMETRY_TEXT_INTERVAL_MS = 500; 
    constexpr uint32_t REVERSE_GEAR_DELAY_MS = 1000;

    // Масштабирование и физика управления
    constexpr float THROTTLE_PERCENT_SCALE = 100.0f;
    constexpr float MAX_POWER_PERCENT = 100.0f;
    
    // Шаги изменения мощности (Slew Rate)
    constexpr uint8_t POWER_RAMP_UP_STEP = 1;
    constexpr uint8_t POWER_RAMP_DOWN_STEP = 2;
    constexpr uint8_t POWER_REVERSE_BRAKE_STEP = 5;

    // Направления движения контроллера FarDriver
    constexpr uint8_t DIRECTION_FORWARD = 0;
    constexpr uint8_t DIRECTION_REVERSE = 1;
}

const AP_Param::GroupInfo AP_FarDriverThrottle::var_info[] = {
    AP_GROUPINFO("MAX_RPM", 1, AP_FarDriverThrottle, max_rpm, 3000),
    AP_GROUPEND
};

AP_FarDriverThrottle::AP_FarDriverThrottle() {
    fardriver_protocol_init();
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_FarDriverThrottle::init(AP_SerialManager &serial_manager) {
    _uart = serial_manager.find_serial(SERIAL_PROTO_FARDRIVER, 0);
    if (_uart != nullptr) {
        _uart->begin(DEFAULT_BAUDRATE); 
    }
}

void AP_FarDriverThrottle::update(float throttle_out) {
    if (_uart == nullptr || !_uart->is_initialized()) {
        return; 
    }

    const uint32_t now = AP_HAL::millis();
    uint32_t available_bytes = _uart->available();
    FardriverTelemetry telemetry_data{};

    if (available_bytes > MAX_RX_BYTES_PER_LOOP) {
        available_bytes = MAX_RX_BYTES_PER_LOOP;
    }
    
    for (uint32_t i = 0; i < available_bytes; i++) {
        const uint8_t b = _uart->read();
        if (fardriver_parse_telemetry(b, &telemetry_data)) {
            
            // Отправляем именованные флоаты для графиков в Mission Planner
            gcs().send_named_float("OUTBM_Volt", telemetry_data.voltage);
            gcs().send_named_float("OUTBM_Curr", telemetry_data.current);
            gcs().send_named_float("OUTBM_Temp", (float)telemetry_data.temperature_mot); // Исправлено имя поля
            gcs().send_named_float("OUTBM_RPM",  (float)telemetry_data.rpm);
            
            // ИСПРАВЛЕНИЕ: Вывод новых линейных и фазных токов на графики GCS телеметрии
            gcs().send_named_float("OUTBM_LCurr", telemetry_data.line_current);
            gcs().send_named_float("OUTBM_PCurr", telemetry_data.phase_current);

            // Текстовый вывод в консоль GCS по таймеру
            static uint32_t last_gcs_text_ms = 0;
            if (now - last_gcs_text_ms >= TELEMETRY_TEXT_INTERVAL_MS) {
                last_gcs_text_ms = now;

                // В лог консоли выводим полные расширенные данные по токам и заряду АКБ
                gcs().send_text(MAV_SEVERITY_INFO, "OUTB_MOT: RPM: %u | Bat: %.1fV | %.1fA | LineCur: %.1fA | PhaseCur: %.1fA | SOC: %d%%", 
                                telemetry_data.rpm, 
                                (double)telemetry_data.voltage, 
                                (double)telemetry_data.current,
                                (double)telemetry_data.line_current,
                                (double)telemetry_data.phase_current,
                                telemetry_data.charge_state);
            }
        }
    }

    // Логика отправки команд управления на FarDriver (раз в SEND_INTERVAL_MS)
    if (now - _last_send_ms < SEND_INTERVAL_MS) {
        return;
    }
    if (_uart->txspace() < MIN_TX_SPACE_REQUIRED) {
        return; 
    }
    _last_send_ms = now;

    uint8_t target_power = 0;
    uint8_t target_dir = DIRECTION_FORWARD;

    if (throttle_out < 0.0f) {
        target_dir = DIRECTION_REVERSE; 
        target_power = (uint8_t)(-throttle_out * THROTTLE_PERCENT_SCALE);
    } else {
        target_dir = DIRECTION_FORWARD; 
        target_power = (uint8_t)(throttle_out * THROTTLE_PERCENT_SCALE);
    }
    
    if (target_power > MAX_POWER_PERCENT) {
        target_power = MAX_POWER_PERCENT;
    }

    static uint8_t CurrentPower = 0;
    static uint8_t CurrentDir = DIRECTION_FORWARD;
    static uint32_t ReverseDelayTimer = 0;

    // Безопасный переход на реверс (сброс газа в 0 -> задержка -> переключение)
    if (CurrentDir != target_dir) {
        if (CurrentPower > 0) {
            CurrentPower = (CurrentPower > POWER_REVERSE_BRAKE_STEP) ? (CurrentPower - POWER_REVERSE_BRAKE_STEP) : 0;
        } else {
            if (ReverseDelayTimer == 0) {
                ReverseDelayTimer = now;
            }
            if (now - ReverseDelayTimer >= REVERSE_GEAR_DELAY_MS) {
                CurrentDir = target_dir;
                ReverseDelayTimer = 0;
            }
        }
    } else {
        ReverseDelayTimer = 0;
        if (CurrentPower < target_power) {
            CurrentPower += POWER_RAMP_UP_STEP;
        } else if (CurrentPower > target_power) {
            CurrentPower = (CurrentPower > POWER_RAMP_DOWN_STEP) ? (CurrentPower - POWER_RAMP_DOWN_STEP) : 0;
        }
    }

    uint8_t tx_packet[PACKET_SIZE_BYTES];
    fardriver_create_power_packet(CurrentPower, CurrentDir, tx_packet);
    _uart->write(tx_packet, PACKET_SIZE_BYTES);
}
