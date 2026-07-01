#include "AP_ModbusSteering.h"
#include <GCS_MAVLink/GCS.h>

extern "C"
{
    void modbus_create_write_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t value, uint8_t *out_buffer);
    void modbus_create_write32_packet(uint8_t slave_id, uint16_t reg_addr, int32_t value, uint8_t *out_buffer);
    void modbus_create_read_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count, uint8_t *out_buffer);
    uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
}

extern const AP_HAL::HAL &hal;

// --- БЛОК РЕГИСТРАЦИИ ПАРАМЕТРОВ В ARDUPILOT ---
const AP_Param::GroupInfo AP_ModbusSteering::var_info[] = {
    // Параметр SLAVE_ID (по умолчанию 1)
    AP_GROUPINFO("SLAVE_ID", 1, AP_ModbusSteering, slave_id, 1),

    // Параметр REG_ADDR (регистр позиции 0x0034 в dec = 52)
    AP_GROUPINFO("REG_ADDR", 2, AP_ModbusSteering, reg_address, 52),

    // Параметр максимального количества шагов руля
    AP_GROUPINFO("MAX_STEPS", 3, AP_ModbusSteering, max_steps, 2000),

    // Стартовая скорость (Регистр 0x0030), дефолт 15 об/мин
    AP_GROUPINFO("START_SPD", 4, AP_ModbusSteering, start_speed, 15),

    // Максимальная скорость (Регистр 0x0033), дефолт 120 об/мин для работы с редуктором
    AP_GROUPINFO("MAX_SPD", 5, AP_ModbusSteering, max_speed, 120),

    AP_GROUPEND};

AP_ModbusSteering::AP_ModbusSteering()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_ModbusSteering::init(AP_SerialManager &serial_manager)
{
    // Поиск выделенного UART-порта (Протокол 101 - Scripting/Custom)
    _uart = serial_manager.find_serial((AP_SerialManager::SerialProtocol)101, 0);
    if (_uart != nullptr)
    {
        _uart->begin(115200);
    }
}

void AP_ModbusSteering::update(float steering_out)
{
    if (_uart == nullptr || !_uart->is_initialized())
    {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    uint32_t available_bytes = _uart->available();

    // --- ВРЕМЕННЫЙ ОТЛАДОЧНЫЙ ВЫВОД ВХОДЯЩЕГО РУЛЯ ---
    static uint32_t last_debug_steer_ms = 0;
    if (now - last_debug_steer_ms >= 2000)
    {
        last_debug_steer_ms = now;
        gcs().send_text(MAV_SEVERITY_INFO, "AP_ModbusSteering: Input steering_out = %.3f", (double)steering_out);
    }
    // -------------------------------------------------

    // Определение состояний конечного автомата инициализации и работы
    enum class DriveState
    {
        INIT_ENABLE,       // 1. Подача питания на обмотки (0x0038 = 1)
        INIT_SUBDIVISION,  // 2. Установка микрошага (0x0023 = 4000 импульсов/оборот)
        INIT_START_SPD,    // 3. Стартовая скорость JOG/Позиции (0x0030)
        INIT_MAX_SPD,      // 4. Ограничение максимальной скорости (0x0033)
        INIT_TIMEOUT,      // 5. Включение защиты по таймауту связи (0x0039 = 500 мс)
        INIT_ACCEL,        // 6. Время разгона в мс (0x0031 = 200 ms)
        INIT_DECEL,        // 7. Время торможения в мс (0x0032 = 200 ms)
        RUN_WRITE_POS,     // 8. Штатная работа: Чистая запись полетной позиции руля
        RUN_READ_TELEMETRY // 9. Штатная работа: Опрос энкодера (0x0007-0x0008)
    };

    static DriveState current_state = DriveState::INIT_ENABLE;
    static uint32_t last_telemetry_rcvd_ms = 0;
    static bool response_received = false;

    // --- БЛОК АППАРАТНОГО ПАРСИНГА ОТВЕТОВ MODBUS ---
    if (available_bytes > 0)
    {
        uint8_t local_buf[64]; // Выделен статический буфер фиксированного размера
        if (available_bytes > 64)
        {
            available_bytes = 64;
        }
        for (uint32_t i = 0; i < available_bytes; i++)
        {
            local_buf[i] = _uart->read();
        }

        // Парсинг ответов для функций записи (0x06 и 0x10 возвращают эхо)
        if (available_bytes >= 8)
        {
            for (uint32_t i = 0; i <= available_bytes - 8; i++)
            {
                if (local_buf[i] == (uint8_t)slave_id.get() && (local_buf[i + 1] == 0x06 || local_buf[i + 1] == 0x10))
                {
                    uint16_t reg = (local_buf[i + 2] << 8) | local_buf[i + 3];
                    uint16_t received_crc = (local_buf[i + 7] << 8) | local_buf[i + 6];

                    if (modbus_crc16(&local_buf[i], 6) == received_crc)
                    {
                        // Проверяем подтверждение текущего шага инициализации от драйвера
                        if (current_state == DriveState::INIT_ENABLE && reg == 0x0038)
                            response_received = true;
                        if (current_state == DriveState::INIT_SUBDIVISION && reg == 0x0023)
                            response_received = true;
                        if (current_state == DriveState::INIT_START_SPD && reg == 0x0030)
                            response_received = true;
                        if (current_state == DriveState::INIT_MAX_SPD && reg == 0x0033)
                            response_received = true;
                        if (current_state == DriveState::INIT_TIMEOUT && reg == 0x0039)
                            response_received = true;
                        if (current_state == DriveState::INIT_ACCEL && reg == 0x0031)
                            response_received = true;
                        if (current_state == DriveState::INIT_DECEL && reg == 0x0032)
                            response_received = true;
                        break;
                    }
                }
            }
        }

        // Парсинг ответа телеметрии энкодера (0x03 возвращает 9 байт)
        if (current_state == DriveState::RUN_READ_TELEMETRY && available_bytes >= 9)
        {
            for (uint32_t i = 0; i <= available_bytes - 9; i++)
            {
                if (local_buf[i] == (uint8_t)slave_id.get() && local_buf[i + 1] == 0x03 && local_buf[i + 2] == 0x04)
                {
                    uint16_t received_crc = (local_buf[i + 8] << 8) | local_buf[i + 7];
                    if (modbus_crc16(&local_buf[i], 7) == received_crc)
                    {
                        uint16_t high_word = (local_buf[i + 3] << 8) | local_buf[i + 4];
                        uint16_t low_word = (local_buf[i + 5] << 8) | local_buf[i + 6];

                        int32_t actual_position = static_cast<int32_t>(((uint32_t)high_word << 16) | low_word);
                        last_telemetry_rcvd_ms = now; // Сброс таймера таймаута связи
                        response_received = true;

                        gcs().send_named_float("OUTB_STEER", (float)actual_position);

                        static uint32_t last_text_ms = 0;
                        if (now - last_text_ms >= 2000)
                        {
                            last_text_ms = now;
                            float max_val = max_steps.get() > 0 ? (float)max_steps.get() : 1.0f;
                            gcs().send_text(MAV_SEVERITY_INFO, "CL57R: Pos: %ld | %.1f%%",
                                            (long)actual_position, (double)((actual_position / max_val) * 100.0f));
                        }
                        break;
                    }
                }
            }
        }
    }

    // --- ЗАЩИТА: Сброс автомата при потере связи (таймаут 1 сек) ---
    if (current_state >= DriveState::RUN_WRITE_POS && (now - last_telemetry_rcvd_ms) > 1000)
    {
        current_state = DriveState::INIT_ENABLE;
        gcs().send_text(MAV_SEVERITY_WARNING, "CL57R: Connection lost! Re-initializing...");
    }

    // --- БЛОК ОТПРАВКИ КОМАНД ПО ИНТЕРВАЛУ (10 Гц = 100 мс) ---
    if ((now - _last_send_ms) < SEND_INTERVAL_MS)
    {
        return;
    }
    if (_uart->txspace() < 22)
    {
        return;
    }
    _last_send_ms = now;
    uint8_t tx_packet[8]; // Буфер фиксированного размера для стандартных Modbus-команд

    // Продвигаем автомат вперед при наличии подтверждения от привода
    if (response_received)
    {
        response_received = false;
        switch (current_state)
        {
        case DriveState::INIT_ENABLE:
            current_state = DriveState::INIT_SUBDIVISION;
            break;
        case DriveState::INIT_SUBDIVISION:
            current_state = DriveState::INIT_START_SPD;
            break;
        case DriveState::INIT_START_SPD:
            current_state = DriveState::INIT_MAX_SPD;
            break;
        case DriveState::INIT_MAX_SPD:
            current_state = DriveState::INIT_TIMEOUT;
            break;
        case DriveState::INIT_TIMEOUT:
            current_state = DriveState::INIT_ACCEL;
            break;
        case DriveState::INIT_ACCEL:
            current_state = DriveState::INIT_DECEL;
            break;
        case DriveState::INIT_DECEL:
            current_state = DriveState::RUN_WRITE_POS;
            last_telemetry_rcvd_ms = now;
            gcs().send_text(MAV_SEVERITY_INFO, "CL57R: All parameters set. Driver READY.");
            break;
        case DriveState::RUN_WRITE_POS:
            current_state = DriveState::RUN_READ_TELEMETRY;
            break;
        case DriveState::RUN_READ_TELEMETRY:
            current_state = DriveState::RUN_WRITE_POS;
            break;
        }
    }

    // Выполнение отправки пакета в зависимости от состояния автомата
    switch (current_state)
    {
        case DriveState::INIT_ENABLE:
            modbus_create_write_packet((uint8_t)slave_id.get(), 0x0038, 0x0001, tx_packet);
            _uart->write(tx_packet, 8);
            break;

        case DriveState::INIT_SUBDIVISION:
            modbus_create_write_packet((uint8_t)slave_id.get(), 0x0023, 4000, tx_packet);
            _uart->write(tx_packet, 8);
            break;

        case DriveState::INIT_START_SPD:
            modbus_create_write_packet((uint8_t)slave_id.get(), 0x0030, (uint16_t)start_speed.get(), tx_packet);
            _uart->write(tx_packet, 8);
            break;

        case DriveState::INIT_MAX_SPD:
            modbus_create_write_packet((uint8_t)slave_id.get(), 0x0033, (uint16_t)max_speed.get(), tx_packet);
            _uart->write(tx_packet, 8);
            break;

        case DriveState::INIT_TIMEOUT:
            // Настройка аппаратного таймаута CL57R в регистр 0x0039 на 500 мс
            modbus_create_write_packet((uint8_t)slave_id.get(), 0x0039, 500, tx_packet);
            _uart->write(tx_packet, 8);
            break;

        case DriveState::INIT_ACCEL:
            modbus_create_write_packet((uint8_t)slave_id.get(), 0x0031, 200, tx_packet);
            _uart->write(tx_packet, 8);
            break;

        case DriveState::INIT_DECEL:
            modbus_create_write_packet((uint8_t)slave_id.get(), 0x0032, 200, tx_packet);
            _uart->write(tx_packet, 8);
            break;

        case DriveState::RUN_WRITE_POS:
        {
            // Ограничиваем входящий полетный сигнал для безопасности от -1.0 до +1.0
            float clean_steering = steering_out;
            if (clean_steering > 1.0f)
                clean_steering = 1.0f;
            if (clean_steering < -1.0f)
                clean_steering = -1.0f;

            // Масштабируем относительный руль [-1.0...1.0] в целевые импульсы мотора
            int32_t target_pulses = (int32_t)(clean_steering * (double)max_steps.get());
            static int32_t last_sent_pulses = 0;
            static bool first_run = true;

            // Вывод отладки в Mission Planner каждые 500 мс, чтобы не спамить в шину
            static uint32_t last_debug_ms = 0;
            uint32_t now_ms = AP_HAL::millis();
            if (now_ms - last_debug_ms >= 500) {
                last_debug_ms = now_ms;
                gcs().send_text(MAV_SEVERITY_INFO, "Modbus: out=%.3f, target=%ld, last=%ld", 
                                (double)clean_steering, (long int)target_pulses, (long int)last_sent_pulses);
            }


            // Зона нечувствительности: шлём команду, только если цель изменилась более чем на 5 импульсов
            // Это полностью устраняет колебания, рывки и дребезг мотора в крайних точках
            if (!first_run && abs(target_pulses - last_sent_pulses) <= 5)
            {
                current_state = DriveState::RUN_READ_TELEMETRY;
                response_received = true; // Быстрый форсированный переход на чтение
                break;
            }
            first_run = false;
            last_sent_pulses = target_pulses;

            uint8_t raw_packet[15]; // Буфер пакета многорегистровой записи (0x10)
            uint8_t idx = 0;

            raw_packet[idx++] = (uint8_t)slave_id.get();
            raw_packet[idx++] = 0x10;
            raw_packet[idx++] = 0x00;
            raw_packet[idx++] = 0x34; // Стартовый регистр позиции (0x0034)
            raw_packet[idx++] = 0x00;
            raw_packet[idx++] = 0x03; // Пишем 3 регистра (0x0034, 0x0035, 0x0036)
            raw_packet[idx++] = 0x06;

            // Разложение 32-битной целевой позиции
            raw_packet[idx++] = (uint8_t)((target_pulses >> 24) & 0xFF);
            raw_packet[idx++] = (uint8_t)((target_pulses >> 16) & 0xFF);
            raw_packet[idx++] = (uint8_t)((target_pulses >> 8) & 0xFF);
            raw_packet[idx++] = (uint8_t)(target_pulses & 0xFF);

            // Регистр 0x0036: Управляющее слово старта абсолютного позиционирования
            raw_packet[idx++] = 0x00;
            raw_packet[idx++] = 0x07;

            // Расчет контрольной суммы
            uint16_t crc = modbus_crc16(raw_packet, idx);
            raw_packet[idx++] = (uint8_t)(crc & 0xFF);
            raw_packet[idx++] = (uint8_t)((crc >> 8) & 0xFF);

            _uart->write(raw_packet, 15);
            break;
        }

        case DriveState::RUN_READ_TELEMETRY:
        {
            // Чтение текущего положения вала (Регистры 0x0007 и 0x0008, читаем 2 слова)
            modbus_create_read_packet((uint8_t)slave_id.get(), 0x0007, 2, tx_packet);
            _uart->write(tx_packet, 8);
            break;
        }
    }
}
