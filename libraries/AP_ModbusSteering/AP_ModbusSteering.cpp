#include "AP_ModbusSteering.h"
#include <GCS_MAVLink/GCS.h>

// Объявляем типы и функции из вашего файла modbus_protocol.c
typedef struct
{
    int32_t actual_position;
    uint16_t error_code;
} StepperTelemetry;

extern "C"
{
    void modbus_create_write_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t value, uint8_t *out_buffer);
    void modbus_create_write_multiple_packet(uint8_t slave_id, uint16_t start_reg, uint16_t reg_count, const uint16_t *reg_values, uint8_t *out_buffer);
    void modbus_create_read_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count, uint8_t *out_buffer);
    uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
    bool modbus_parse_read_response(uint8_t byte, uint8_t expected_bytes, StepperTelemetry *out_telemetry);
}

extern const AP_HAL::HAL &hal;

namespace {
constexpr uint16_t REG_STATUS_WORD = 0x0003;
constexpr uint16_t REG_ENCODER_POS = 0x0007;
constexpr uint16_t REG_AUX_CONTROL = 0x0037;
constexpr uint16_t AUX_ALARM_CLEAR = 0x0004;

// Позиция: WRITE + READ_POS = 2 шага → 10 Гц при 50 мс на шаг.
constexpr uint32_t POSITION_LOOP_HZ = 10;
constexpr uint32_t POSITION_SEND_INTERVAL_MS = 1000 / (POSITION_LOOP_HZ * 2);

// Статус/ошибки: раз в 10 циклов позиции → 1 Гц.
constexpr uint32_t STATUS_READ_HZ = 1;
constexpr uint8_t STATUS_READ_EVERY_N_CYCLES = POSITION_LOOP_HZ / STATUS_READ_HZ;

const char *cl57r_error_str(uint16_t code)
{
    switch (code) {
    case 0:
        return "OK";
    case 1:
        return "Overcurrent";
    case 2:
        return "Overvoltage";
    case 4:
        return "Tracking error";
    default:
        return "Unknown";
    }
}
} // namespace

// --- БЛОК РЕГИСТРАЦИИ ПАРАМЕТРОВ С ПРАВИЛЬНЫМИ ХЕШ-КОММЕНТАРИЯМИ ---
// @Group: STEER_
// @Path: AP_ModbusSteering.cpp
const AP_Param::GroupInfo AP_ModbusSteering::var_info[] = {
    // @Param: SLAVE_ID
    // @DisplayName: Modbus Slave ID
    // @Description: ID драйвера CL57R в сети Modbus
    // @User: Standard
    AP_GROUPINFO("SLAVE_ID", 1, AP_ModbusSteering, slave_id, 1),

    // @Param: REG_ADDR
    // @DisplayName: Base Register Address
    // @Description: Регистр позиции 0x0034 в dec = 52
    // @User: Advanced
    AP_GROUPINFO("REG_ADDR", 2, AP_ModbusSteering, reg_address, 52),

    // @Param: MAX_STEPS
    // @DisplayName: Maximum Steering Steps
    // @Description: Максимальный рабочий диапазон руля в импульсах. Для 4-х оборотов при микрошаге 4000 установите 16000.
    // @User: Standard
    AP_GROUPINFO("MAX_STEPS", 3, AP_ModbusSteering, max_steps, 16000),

    // @Param: START_SPD
    // @DisplayName: Start JOG Speed
    // @Description: Стартовая скорость JOG в RPM (Регистр 0x0030)
    // @User: Standard
    AP_GROUPINFO("START_SPD", 4, AP_ModbusSteering, start_speed, 15),

    // @Param: MAX_SPD
    // @DisplayName: Maximum Speed RPM
    // @Description: Максимальная рабочая скорость в об/мин (Регистр 0x0033)
    // @User: Standard
    AP_GROUPINFO("MAX_SPD", 5, AP_ModbusSteering, max_speed, 120),

    AP_GROUPEND};

AP_ModbusSteering::AP_ModbusSteering()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_ModbusSteering::init(AP_SerialManager &serial_manager)
{
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

    // Определение состояний конечного автомата
    enum class DriveState
    {
        INIT_ENABLE,       // 0
        INIT_CLEAR_ALARM,  // 1
        INIT_SUBDIVISION,  // 2
        INIT_START_SPD,    // 3
        INIT_MAX_SPD,      // 4
        INIT_ACCEL,        // 5
        INIT_DECEL,        // 6
        RUN_WRITE_POS,     // 7
        RUN_READ_POS,      // 8
        RUN_READ_STATUS,   // 9
    };

    static DriveState current_state = DriveState::INIT_ENABLE;
    static uint32_t last_telemetry_rcvd_ms = 0;
    static bool response_received = false;
    static int32_t debug_target_pulses = 0;
    static int32_t debug_actual_pulses = 0;
    static uint16_t last_driver_error_code = 0;
    static uint16_t last_driver_status_word = 0;
    static uint8_t position_cycles_since_status = 0;

    // --- БЛОК АППАРАТНОГО ПАРСИНГА ОТВЕТОВ ВНУТРИ C++ ---
    if (available_bytes > 0)
    {
        uint8_t local_buf[64];
        if (available_bytes > 64)
        {
            available_bytes = 64;
        }
        for (uint32_t i = 0; i < available_bytes; i++)
        {
            local_buf[i] = _uart->read();
        }

        // 1. Парсинг эха для шагов инициализации (строгая проверка регистров)
        if (current_state < DriveState::RUN_WRITE_POS && available_bytes >= 8)
        {
            for (uint32_t i = 0; i <= available_bytes - 8; i++)
            {
                if (local_buf[i] == (uint8_t)slave_id.get())
                {
                    uint16_t reg = (local_buf[i + 2] << 8) | local_buf[i + 3];
                    uint16_t received_crc = (local_buf[i + 7] << 8) | local_buf[i + 6];
                    if (modbus_crc16(&local_buf[i], 6) == received_crc)
                    {
                        if (current_state == DriveState::INIT_ENABLE && reg == 0x0038)
                            response_received = true;
                        if (current_state == DriveState::INIT_CLEAR_ALARM && reg == REG_AUX_CONTROL)
                            response_received = true;
                        if (current_state == DriveState::INIT_SUBDIVISION && reg == 0x0023)
                            response_received = true;
                        if (current_state == DriveState::INIT_START_SPD && reg == 0x0030)
                            response_received = true;
                        if (current_state == DriveState::INIT_MAX_SPD && reg == 0x0033)
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

        // 2. Парсинг позиции энкодера (0x0007-0x0008, функция 0x03, 9 байт)
        if (current_state == DriveState::RUN_READ_POS && available_bytes >= 9)
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

                        last_telemetry_rcvd_ms = now;
                        response_received = true;

                        debug_actual_pulses = actual_position;
                        gcs().send_debug_vect("STEER",
                                              (float)debug_actual_pulses,
                                              (float)debug_target_pulses,
                                              (float)(debug_target_pulses - debug_actual_pulses));
                        break;
                    }
                }
            }
        }

        // 3. Парсинг статуса и кода ошибки драйвера (0x0003-0x0004)
        if (current_state == DriveState::RUN_READ_STATUS && available_bytes >= 9)
        {
            for (uint32_t i = 0; i <= available_bytes - 9; i++)
            {
                if (local_buf[i] == (uint8_t)slave_id.get() && local_buf[i + 1] == 0x03 && local_buf[i + 2] == 0x04)
                {
                    uint16_t received_crc = (local_buf[i + 8] << 8) | local_buf[i + 7];
                    if (modbus_crc16(&local_buf[i], 7) == received_crc)
                    {
                        const uint16_t status_word = (local_buf[i + 3] << 8) | local_buf[i + 4];
                        const uint16_t error_code = (local_buf[i + 5] << 8) | local_buf[i + 6];

                        last_telemetry_rcvd_ms = now;
                        response_received = true;

                        if (error_code != last_driver_error_code) {
                            if (error_code != 0) {
                                gcs().send_text(MAV_SEVERITY_WARNING,
                                                "CL57R: error 0x%04X (%s) status=0x%04X",
                                                error_code,
                                                cl57r_error_str(error_code),
                                                status_word);
                            } else if (last_driver_error_code != 0) {
                                gcs().send_text(MAV_SEVERITY_INFO, "CL57R: alarm cleared");
                            }
                            last_driver_error_code = error_code;
                        }

                        if (status_word != last_driver_status_word) {
                            gcs().send_text(MAV_SEVERITY_INFO,
                                            "CL57R: status 0x%04X (alarm=%u enabled=%u)",
                                            status_word,
                                            (unsigned)((status_word >> 3) & 1),
                                            (unsigned)((status_word >> 4) & 1));
                            last_driver_status_word = status_word;
                        }
                        break;
                    }
                }
            }
        }
    }

    // Защита: Сброс автомата при потере связи в рабочем режиме (таймаут 2 секунды)
    if (current_state >= DriveState::RUN_WRITE_POS && (now - last_telemetry_rcvd_ms) > 2000)
    {
        current_state = DriveState::INIT_ENABLE;
        last_driver_error_code = 0;
        last_driver_status_word = 0;
        position_cycles_since_status = 0;
        gcs().send_text(MAV_SEVERITY_WARNING, "CL57R: Modbus Timeout! Re-initializing...");
    }

    // --- БЛОК ОТПРАВКИ КОМАНД ПО ТАЙМЕРУ (позиция 10 Гц, статус 1 Гц) ---
    if ((now - _last_send_ms) < POSITION_SEND_INTERVAL_MS)
    {
        return;
    }
    if (_uart->txspace() < 22)
    {
        return;
    }
    _last_send_ms = now;
    uint8_t tx_packet[16]; // Задан фиксированный размер массива на стеке

    // Продвигаем конечный автомат вперед
    if (response_received)
    {
        response_received = false;
        switch (current_state)
        {
        case DriveState::INIT_ENABLE:
            current_state = DriveState::INIT_CLEAR_ALARM;
            break;
        case DriveState::INIT_CLEAR_ALARM:
            current_state = DriveState::INIT_SUBDIVISION;
            break;
        case DriveState::INIT_SUBDIVISION:
            current_state = DriveState::INIT_START_SPD;
            break;
        case DriveState::INIT_START_SPD:
            current_state = DriveState::INIT_MAX_SPD;
            break;
        case DriveState::INIT_MAX_SPD:
            current_state = DriveState::INIT_ACCEL;
            break; // Прыгаем сразу на разгон (без таймаута)
        case DriveState::INIT_ACCEL:
            current_state = DriveState::INIT_DECEL;
            break;
        case DriveState::INIT_DECEL:
            current_state = DriveState::RUN_WRITE_POS;
            last_telemetry_rcvd_ms = now;
            position_cycles_since_status = 0;
            gcs().send_text(MAV_SEVERITY_INFO, "CL57R: Modbus Driver READY.");
            break;
        case DriveState::RUN_WRITE_POS:
            current_state = DriveState::RUN_READ_POS;
            break;
        case DriveState::RUN_READ_POS:
            position_cycles_since_status++;
            if (position_cycles_since_status >= STATUS_READ_EVERY_N_CYCLES) {
                position_cycles_since_status = 0;
                current_state = DriveState::RUN_READ_STATUS;
            } else {
                current_state = DriveState::RUN_WRITE_POS;
            }
            break;
        case DriveState::RUN_READ_STATUS:
            current_state = DriveState::RUN_WRITE_POS;
            break;
        }
    }

    // Отправка пакетов на основе текущего состояния
    switch (current_state)
    {
    case DriveState::INIT_ENABLE:
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0038, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        break;

    case DriveState::INIT_CLEAR_ALARM:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_AUX_CONTROL, AUX_ALARM_CLEAR, tx_packet);
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

    case DriveState::INIT_ACCEL:
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0031, 200, tx_packet);
        _uart->write(tx_packet, 8);
        break;

    case DriveState::INIT_DECEL:
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0032, 200, tx_packet);
        _uart->write(tx_packet, 8);
        break;

         case DriveState::RUN_WRITE_POS: {
            float clean_steering = steering_out;
            if (clean_steering > 1.0f)  clean_steering = 1.0f;
            if (clean_steering < -1.0f) clean_steering = -1.0f;
 
            // Масштабируем относительный руль в целевые шаги (до 16000 импульсов)
            int32_t target_pulses = (int32_t)(clean_steering * 16000.0);
            debug_target_pulses = target_pulses;
 
            // ИСПРАВЛЕНО: Объявляем values строго как массив из 3-х элементов
            uint16_t values[3];
            values[0] = (uint16_t)((target_pulses >> 16) & 0xFFFF); // 0x0034 Старшее слово позиции
            values[1] = (uint16_t)(target_pulses & 0xFFFF);         // 0x0035 Младшее слово позиции
            values[2] = 0x0007;                                     // 0x0036 Проверенный рабочий триггер
 
            // Теперь имя массива автоматически передастся как указатель (const uint16_t*)
            modbus_create_write_multiple_packet((uint8_t)slave_id.get(), 0x0034, 3, values, tx_packet);
            
            // Длина пакета функции 0x10 при отправке 3 регистров всегда 15 байт
            _uart->write(tx_packet, 15);

            // Явно переводим автомат в режим чтения позиции
            current_state = DriveState::RUN_READ_POS;
            break;
        }


    case DriveState::RUN_READ_POS:
    {
        modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
        _uart->write(tx_packet, 8);
        break;
    }

    case DriveState::RUN_READ_STATUS:
    {
        modbus_create_read_packet((uint8_t)slave_id.get(), REG_STATUS_WORD, 2, tx_packet);
        _uart->write(tx_packet, 8);
        break;
    }
    }
}