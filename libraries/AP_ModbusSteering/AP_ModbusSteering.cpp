#include "AP_ModbusSteering.h"
#include <GCS_MAVLink/GCS.h>

extern "C" {
    void modbus_create_write_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t value, uint8_t *out_buffer);
    void modbus_create_write32_packet(uint8_t slave_id, uint16_t reg_addr, int32_t value, uint8_t *out_buffer);
    void modbus_create_read_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count, uint8_t *out_buffer);
    uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
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

    const uint32_t now = AP_HAL::millis();
    uint32_t available_bytes = _uart->available();
    
    // Переменные для отслеживания состояния инициализации
    static enum class InitState {
        LOCK_COILS,     // Циклическая отправка запроса на блокировку обмоток
        READY           // Обмотки успешно заблокированы, переход к штатной работе
    } init_state = InitState::LOCK_COILS;

    static bool lock_confirmed = false;

    // БРОНЕБОЙНЫЙ РАЗБОР: Читаем доступный буфер за один проход цикла update
    if (available_bytes > 0) {
        uint8_t local_buf[64];
        if (available_bytes > 64) {
            available_bytes = 64;
        }
        
        for (uint32_t i = 0; i < available_bytes; i++) {
            local_buf[i] = _uart->read();
        }

        // 1. ПАРСИНГ ПОДТВЕРЖДЕНИЯ БЛОКИРОВКИ (Функция 0x06 возвращает 8-байтовый эхо-ответ)
        if (init_state == InitState::LOCK_COILS && available_bytes >= 8) {
            for (uint32_t i = 0; i <= available_bytes - 8; i++) {
                if (local_buf[i] == (uint8_t)slave_id.get() && local_buf[i+1] == 0x06) {
                    uint16_t reg = (local_buf[i+2] << 8) | local_buf[i+3];
                    uint16_t val = (local_buf[i+4] << 8) | local_buf[i+5];
                    uint16_t received_crc = (local_buf[i+7] << 8) | local_buf[i+6];
                    
                    if (reg == 0x0038 && val == 0x0001 && modbus_crc16(&local_buf[i], 6) == received_crc) {
                        lock_confirmed = true;
                        init_state = InitState::READY;
                        gcs().send_text(MAV_SEVERITY_INFO, "OUTB_STEER: Coils locked successfully!");
                        break;
                    }
                }
            }
        }

        // 2. ШТАТНЫЙ ПАРСИНГ ТЕЛЕМЕТРИИ ЭНКОДЕРА (Функция 0x03, ответ 9 байт)
        if (init_state == InitState::READY && available_bytes >= 9) {
            for (uint32_t i = 0; i <= available_bytes - 9; i++) {
                if (local_buf[i] == (uint8_t)slave_id.get() && local_buf[i+1] == 0x03 && local_buf[i+2] == 0x04) {
                    
                    uint16_t received_crc = (local_buf[i+8] << 8) | local_buf[i+7];
                    uint16_t calculated_crc = modbus_crc16(&local_buf[i], 7); 

                    if (received_crc == calculated_crc) {
                        uint16_t high_word = (local_buf[i+3] << 8) | local_buf[i+4];
                        uint16_t low_word  = (local_buf[i+5] << 8) | local_buf[i+6];
                        
                        uint32_t combined = ((uint32_t)high_word << 16) | low_word;
                        int32_t actual_position = static_cast<int32_t>(combined);

                        gcs().send_named_float("OUTB_STEER", (float)actual_position);

                        static uint32_t last_steer_text_ms = 0;
                        if (now - last_steer_text_ms >= 2000) {
                            last_steer_text_ms = now;
                            
                            float max_val = max_steps.get() > 0 ? (float)max_steps.get() : 1.0f;
                            float steer_pct = ((float)actual_position / max_val) * 100.0f;

                            gcs().send_text(MAV_SEVERITY_INFO, "OUTB_STEER: Pos: %ld steps | %.1f%%", 
                                            (long)actual_position, (double)steer_pct);
                        }
                        break; 
                    }
                }
            }
        }
    }

    // --- Логика отправки команд управления (20 Гц = каждые 50 мс) ---
    if ((now - _last_send_ms) < SEND_INTERVAL_MS) {
        return;
    }
    if (_uart->txspace() < 22) {
        return;
    }
    _last_send_ms = now;

    uint8_t tx_packet[16]; 

    // СОСТОЯНИЕ 1: Ждем жесткого подтверждения фиксации вала от железа
    if (init_state == InitState::LOCK_COILS) {
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0038, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        return; 
    }

    // СОСТОЯНИЕ 2: Штатный цикл работы (Запись позиции / Чтение положения)
    static uint8_t query_toggle = 0;

    if (query_toggle == 0) {
        if (steering_out < -1.0f) steering_out = -1.0f;
        if (steering_out > 1.0f)  steering_out = 1.0f;

        // Шаг А: Запись целевой позиции (Регистр 0x0034, 13 байт)
        int32_t target_pulses = (int32_t)(steering_out * max_steps.get());
        modbus_create_write32_packet((uint8_t)slave_id.get(), 0x0034, target_pulses, tx_packet);
        _uart->write(tx_packet, 13);

        // Шаг Б: Триггер абсолютного движения (Регистр 0x0036 = 0x0003, 8 байт)
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0036, 0x0003, tx_packet);
        _uart->write(tx_packet, 8);

        query_toggle = 1; 
    } 
    else {
        // Запрос текущего значения энкодера (Регистр 0x0007, 8 байт)
        modbus_create_read_packet((uint8_t)slave_id.get(), 0x0007, 2, tx_packet);
        _uart->write(tx_packet, 8);
        
        query_toggle = 0; 
    }
}
