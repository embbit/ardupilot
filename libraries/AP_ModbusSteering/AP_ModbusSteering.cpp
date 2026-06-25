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

    uint32_t now = AP_HAL::millis();
    uint32_t available_bytes = _uart->available();
    
    // БРОНЕБОЙНЫЙ РАЗБОР: Читаем весь доступный буфер за один раз
    if (available_bytes > 0) {
        uint8_t local_buf[64];
        if (available_bytes > 64) {
            available_bytes = 64;
        }
        
        for (uint32_t i = 0; i < available_bytes; i++) {
            local_buf[i] = _uart->read();
        }

         // ОКОНЧАТЕЛЬНОЕ ИСПРАВЛЕНИЕ: Реальный ответ функции 0x03 (4 байта данных) занимает ровно 9 байт!
        if (available_bytes >= 9) {
            for (uint32_t i = 0; i <= available_bytes - 9; i++) {
                if (local_buf[i] == (uint8_t)slave_id.get() && local_buf[i+1] == 0x03 && local_buf[i+2] == 0x04) {
                    
                    // Контрольная сумма CRC-16 для 9-байтового пакета находится в байтах i+7 и i+8
                    // Сама функция modbus_crc16 должна посчитать сумму по первым 7 байтам пакета!
                    uint16_t received_crc = (local_buf[i+8] << 8) | local_buf[i+7];
                    uint16_t calculated_crc = modbus_crc16(&local_buf[i], 7); 

                    if (received_crc == calculated_crc) {
                        // Точная сборка 32-битного значения в формате Leadshine CL57R (Low-Word First)
                        // Младшее слово (Low Word) лежит в байтах i+3 и i+4
                        uint16_t low_word  = (local_buf[i+3] << 8) | local_buf[i+4];
                        
                        // Старшее слово (High Word) лежит в байтах i+5 и i+6
                        uint16_t high_word = (local_buf[i+5] << 8) | local_buf[i+6];
                        
                        // Склеиваем в единую 32-битную беззнаковую переменную
                        uint32_t combined = ((uint32_t)high_word << 16) | low_word;
                        
                        // Восстанавливаем знак для поддержки отрицательных шагов (поворот влево)
                        int32_t actual_position = 0;
                        if (combined > 2147483647) {
                            actual_position = (int32_t)combined - 4294967296;
                        } else {
                            actual_position = (int32_t)combined;
                        }

                        // 1. Отправляем график позиции в Mission Planner
                        gcs().send_named_float("ST_Pos", (float)actual_position);

                        // 2. ВЫВОД В КОНСОЛЬ: Отображаем статус руля в зеленом окне раз в 2000 мс
                        static uint32_t last_steer_text_ms = 0;
                        if (now - last_steer_text_ms >= 2000) {
                            last_steer_text_ms = now;
                            
                            float max_val = max_steps.get() > 0 ? (float)max_steps.get() : 1.0f;
                            float steer_pct = ((float)actual_position / max_val) * 100.0f;

                            gcs().send_text(MAV_SEVERITY_INFO, "CL57R: Pos: %ld steps | %.1f%%", 
                                            (long)actual_position, 
                                            (double)steer_pct);
                        }
                        break; // Валидный пакет успешно обработан, выходим из цикла сканирования
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

    static bool is_motor_enabled = false;
    static uint8_t query_toggle = 0; 
    uint8_t tx_packet[16]; 

    if (!is_motor_enabled) {
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0038, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        is_motor_enabled = true;
        
        gcs().send_text(MAV_SEVERITY_INFO, "CL57R: Steering motor coils are locked!");
        return; 
    }

    if (query_toggle == 0) {
        if (steering_out < -1.0f) steering_out = -1.0f;
        if (steering_out > 1.0f)  steering_out = 1.0f;

        // Шаг А: Запись целевой позиции (13 байт)
        int32_t target_pulses = (int32_t)(steering_out * max_steps.get());
        modbus_create_write32_packet((uint8_t)slave_id.get(), 0x0034, target_pulses, tx_packet);
        _uart->write(tx_packet, 13);

        // Шаг Б: Триггер старта (8 байт)
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0036, 0x0007, tx_packet);
        _uart->write(tx_packet, 8);

        query_toggle = 1; 
    } 
    else {
        // Запрос чтения текущей позиции (0x001C, длина 2 регистра)
        modbus_create_read_packet((uint8_t)slave_id.get(), 0x001C, 2, tx_packet);
        _uart->write(tx_packet, 8);
        
        query_toggle = 0; 
    }
}
