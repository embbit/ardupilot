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

constexpr int32_t CL57R_STEPS_PER_REV = 4000;

namespace {
constexpr uint16_t REG_STATUS_WORD = 0x0003;
constexpr uint16_t REG_ENCODER_POS = 0x0007;
constexpr uint16_t REG_MOTION_CTRL = 0x0036;
constexpr uint16_t REG_AUX_CONTROL = 0x0037;
constexpr uint16_t REG_MOTOR_ENABLE = 0x0038;
constexpr uint16_t REG_TRIGGER_MODE = 0x0039;
constexpr uint16_t REG_POS_MODE = 0x003A;
constexpr uint16_t REG_TRACK_ERR = 0x0052;
constexpr uint16_t AUX_ALARM_CLEAR = 0x0004;
constexpr uint16_t MOTION_START_ABS = 0x0003;  // start + absolute (interrupt via 0x0039)
constexpr uint16_t CL57R_TRACK_ERR_DEFAULT = 8000;
constexpr uint32_t TRACK_ERR_PAUSE_MS = 500;

// Позиция: уставка и чтение энкодера чередуются по таймеру (не ждём ответ для записи).
constexpr uint32_t POSITION_LOOP_HZ = 20;
constexpr uint32_t POSITION_SEND_INTERVAL_MS = 1000 / POSITION_LOOP_HZ;

// Статус/ошибки: раз в 10 опросов энкодера (~1 Гц при 10 Гц опроса).
constexpr uint8_t STATUS_READ_EVERY_N_READS = 10;

constexpr float STICK_CENTER_THRESHOLD = 0.05f;
constexpr uint32_t MODBUS_LINK_TIMEOUT_MS = 3000;

static void modbus_note_link_rx(uint32_t now_ms, uint32_t &link_deadline_ms)
{
    link_deadline_ms = now_ms + MODBUS_LINK_TIMEOUT_MS;
}

static int32_t clamp_int32(int32_t value, int32_t min_val, int32_t max_val)
{
    if (value < min_val) {
        return min_val;
    }
    if (value > max_val) {
        return max_val;
    }
    return value;
}

static int32_t abs_int32(int32_t value)
{
    return (value >= 0) ? value : -value;
}

static int32_t speed_move_limit_pulses(uint16_t rpm, uint32_t cycle_ms)
{
    const int32_t pulses_per_sec = (int32_t)rpm * CL57R_STEPS_PER_REV / 60;
    return MAX(pulses_per_sec * (int32_t)cycle_ms / 1000, 1);
}

static int32_t step_toward(int32_t actual, int32_t target, int32_t max_step)
{
    const int32_t delta = target - actual;
    if (delta > max_step) {
        return actual + max_step;
    }
    if (delta < -max_step) {
        return actual - max_step;
    }
    return target;
}

static int32_t brake_zone_pulses(uint16_t rpm, int32_t active_limit, int32_t max_pulses)
{
    const int32_t pulses_per_sec = (int32_t)rpm * CL57R_STEPS_PER_REV / 60;
    constexpr uint32_t DECEL_MS = 400;
    const int32_t brake_distance = pulses_per_sec * (int32_t)DECEL_MS / 2000;
    return MAX(brake_distance, MAX(active_limit * 8, max_pulses / 8));
}

static int32_t decode_encoder_position(uint16_t high_word, uint16_t low_word)
{
    return (int32_t)(((int32_t)(int16_t)high_word << 16) | low_word);
}

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

int32_t AP_ModbusSteering::travel_limit_pulses() const
{
    if (out_rev.get() > 0 && ratio.get() > 0) {
        return (int32_t)out_rev.get() * ratio.get() * CL57R_STEPS_PER_REV / 2;
    }
    return max_steps.get();
}

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
    // @Description: Импульсы на полный ход стика (±1), если OUT_REV=0. Иначе используется OUT_REV*RATIO*4000/2.
    // @User: Standard
    AP_GROUPINFO("MAX_STEPS", 3, AP_ModbusSteering, max_steps, 200000),

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

    // @Param: POS_DB
    // @DisplayName: Position Deadband
    // @Description: Не повторять команду позиции, если изменение уставки меньше этого порога (импульсы).
    // @Units: pulses
    // @Range: 0 50000
    // @User: Standard
    AP_GROUPINFO("POS_DB", 6, AP_ModbusSteering, pos_db, 500),

    // @Param: RET_SLEW
    // @DisplayName: Return-To-Zero Slew Limit
    // @Description: Макс. шаг при |стик|<5% (возврат в ноль). 0 = та же скорость, что активный ход (MAX_SPD).
    // @Units: pulses
    // @Range: 0 50000
    // @User: Standard
    AP_GROUPINFO("RET_SLEW", 7, AP_ModbusSteering, ret_slew, 0),

    // @Param: OUT_REV
    // @DisplayName: Rudder Lock-to-Lock Turns
    // @Description: Обороты на выходе редуктора упор-упор. При >0 лимит = OUT_REV*RATIO*4000/2 имп на стик.
    // @Units: rev
    // @Range: 0 20
    // @User: Standard
    AP_GROUPINFO("OUT_REV", 8, AP_ModbusSteering, out_rev, 4),

    // @Param: RATIO
    // @DisplayName: Gearbox Ratio
    // @Description: Передаточное число редуктора (об мотора на 1 об выхода).
    // @Range: 1 200
    // @User: Standard
    AP_GROUPINFO("RATIO", 9, AP_ModbusSteering, ratio, 25),

    // @Param: INV
    // @DisplayName: Invert Stick Sign
    // @Description: Инвертировать знак стика относительно энкодера. 1 = стик влево уменьшает импульсы.
    // @Values: 0:Normal,1:Inverted
    // @User: Standard
    AP_GROUPINFO("INV", 10, AP_ModbusSteering, invert, 1),

    // @Param: ACCEL_MS
    // @DisplayName: Acceleration time (ms)
    // @Description: Время разгона CL57R от 0 до MAX_SPD (регистр 0x0031). Увеличьте при рывках на старте.
    // @Units: ms
    // @Range: 50 5000
    // @User: Standard
    AP_GROUPINFO("ACCEL_MS", 11, AP_ModbusSteering, accel_ms, 400),

    // @Param: DECEL_MS
    // @DisplayName: Deceleration time (ms)
    // @Description: Время торможения CL57R от MAX_SPD до 0 (регистр 0x0032). Уменьшите при перелёте упора.
    // @Units: ms
    // @Range: 50 5000
    // @User: Standard
    AP_GROUPINFO("DECEL_MS", 12, AP_ModbusSteering, decel_ms, 400),

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
        INIT_TRACK_ERR,    // 3
        INIT_START_SPD,    // 4
        INIT_MAX_SPD,      // 5
        INIT_ACCEL,        // 6
        INIT_DECEL,        // 7
        INIT_ABS_MODE,     // 8
        INIT_TRIG_MODE,    // 9
        RUN_ACTIVE,        // 10
        TRACK_CLEAR,       // 11
        FAULT_RELEASE,     // 12
        FAULT_LATCHED,     // 13
    };

    static DriveState current_state = DriveState::INIT_ENABLE;
    static uint32_t link_deadline_ms = 0;
    static bool modbus_link_ok = false;
    static bool response_received = false;
    static int32_t debug_target_pulses = 0;
    static int32_t debug_actual_pulses = 0;
    static uint16_t last_driver_error_code = 0;
    static uint16_t last_driver_status_word = 0;
    static int32_t last_sent_target_pulses = 0;
    static bool have_sent_target = false;
    static bool encoder_fault_latched = false;
    static bool pending_track_clear = false;
    static uint32_t tracking_pause_until_ms = 0;
    static bool have_actual_position = false;
    static bool need_position_sync = false;
    static int32_t last_valid_actual = 0;
    static bool have_valid_actual = false;
    static uint8_t run_status_div = 0;
    static bool expecting_status_read = false;
    static bool pending_encoder_read = false;
    static bool run_phase_write = true;

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
        if ((current_state < DriveState::RUN_ACTIVE || current_state == DriveState::FAULT_RELEASE ||
             current_state == DriveState::TRACK_CLEAR) && available_bytes >= 8)
        {
            for (uint32_t i = 0; i <= available_bytes - 8; i++)
            {
                if (local_buf[i] == (uint8_t)slave_id.get())
                {
                    uint16_t reg = (local_buf[i + 2] << 8) | local_buf[i + 3];
                    uint16_t received_crc = (local_buf[i + 7] << 8) | local_buf[i + 6];
                    if (modbus_crc16(&local_buf[i], 6) == received_crc)
                    {
                        modbus_note_link_rx(now, link_deadline_ms);

                        if (current_state == DriveState::INIT_ENABLE && reg == REG_MOTOR_ENABLE)
                            response_received = true;
                        if (current_state == DriveState::FAULT_RELEASE && reg == REG_MOTOR_ENABLE)
                            response_received = true;
                        if (current_state == DriveState::TRACK_CLEAR && reg == REG_AUX_CONTROL)
                            response_received = true;
                        if (current_state == DriveState::INIT_CLEAR_ALARM && reg == REG_AUX_CONTROL)
                            response_received = true;
                        if (current_state == DriveState::INIT_SUBDIVISION && reg == 0x0023)
                            response_received = true;
                        if (current_state == DriveState::INIT_TRACK_ERR && reg == REG_TRACK_ERR)
                            response_received = true;
                        if (current_state == DriveState::INIT_START_SPD && reg == 0x0030)
                            response_received = true;
                        if (current_state == DriveState::INIT_MAX_SPD && reg == 0x0033)
                            response_received = true;
                        if (current_state == DriveState::INIT_ACCEL && reg == 0x0031)
                            response_received = true;
                        if (current_state == DriveState::INIT_DECEL && reg == 0x0032)
                            response_received = true;
                        if (current_state == DriveState::INIT_ABS_MODE && reg == REG_POS_MODE)
                            response_received = true;
                        if (current_state == DriveState::INIT_TRIG_MODE && reg == REG_TRIGGER_MODE)
                            response_received = true;
                        break;
                    }
                }
            }
        }

        // 1b. Эхо записи позиции в RUN (функция 0x10, 8 байт)
        if (current_state == DriveState::RUN_ACTIVE && available_bytes >= 8)
        {
            for (uint32_t i = 0; i <= available_bytes - 8; i++)
            {
                if (local_buf[i] == (uint8_t)slave_id.get() && local_buf[i + 1] == 0x10)
                {
                    uint16_t received_crc = (local_buf[i + 7] << 8) | local_buf[i + 6];
                    if (modbus_crc16(&local_buf[i], 6) == received_crc)
                    {
                        modbus_note_link_rx(now, link_deadline_ms);
                        break;
                    }
                }
            }
        }

        // 2. Парсинг позиции энкодера (0x0007-0x0008, функция 0x03, 9 байт)
        if (((current_state == DriveState::RUN_ACTIVE && pending_encoder_read) ||
             current_state == DriveState::FAULT_LATCHED) && available_bytes >= 9)
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

                        const int32_t actual_position = decode_encoder_position(high_word, low_word);
                        const int32_t max_jump = speed_move_limit_pulses((uint16_t)max_speed.get(),
                                                                          POSITION_SEND_INTERVAL_MS * 2) * 4;

                        if (have_valid_actual &&
                            abs_int32(actual_position - last_valid_actual) > max_jump) {
                            modbus_note_link_rx(now, link_deadline_ms);
                            pending_encoder_read = false;
                            break;
                        }

                        last_valid_actual = actual_position;
                        have_valid_actual = true;
                        modbus_note_link_rx(now, link_deadline_ms);

                        debug_actual_pulses = actual_position;
                        have_actual_position = true;

                        const int32_t max_pulses = travel_limit_pulses();
                        // Разрешаем небольшой перелёт равный TRACK_ERR (8000 имп) —
                        // нормальная погрешность остановки CL57R. Fault только при
                        // реальном уходе за лимит, не из-за jitter энкодера.
                        const int32_t fault_limit = max_pulses + CL57R_TRACK_ERR_DEFAULT;
                        if (abs_int32(actual_position) > fault_limit) {
                            if (!encoder_fault_latched) {
                                encoder_fault_latched = true;
                                current_state = DriveState::FAULT_RELEASE;
                                gcs().send_text(MAV_SEVERITY_CRITICAL,
                                                "CL57R: encoder %ld out of range (+/-%ld), latched",
                                                (long)actual_position,
                                                (long)fault_limit);
                            }
                            break;
                        }

                        response_received = false;
                        pending_encoder_read = false;

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
        if (current_state == DriveState::RUN_ACTIVE && expecting_status_read && available_bytes >= 9)
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

                        modbus_note_link_rx(now, link_deadline_ms);
                        expecting_status_read = false;
                        pending_encoder_read = false;
                        response_received = false;

                        if (error_code != last_driver_error_code) {
                            if (error_code != 0) {
                                gcs().send_text(MAV_SEVERITY_WARNING,
                                                "CL57R: error 0x%04X (%s) status=0x%04X",
                                                error_code,
                                                cl57r_error_str(error_code),
                                                status_word);
                                if (error_code == 4) {
                                    pending_track_clear = true;
                                    tracking_pause_until_ms = now + TRACK_ERR_PAUSE_MS;
                                    need_position_sync = true;
                                }
                            } else if (last_driver_error_code != 0) {
                                gcs().send_text(MAV_SEVERITY_INFO, "CL57R: alarm cleared");
                            }
                            last_driver_error_code = error_code;
                        }

                        if (status_word != last_driver_status_word) {
                            const bool alarm_now = (status_word >> 3) & 1;
                            const bool alarm_was = (last_driver_status_word >> 3) & 1;
                            if (alarm_now != alarm_was) {
                                gcs().send_text(alarm_now ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO,
                                                "CL57R: alarm %s status=0x%04X",
                                                alarm_now ? "ON" : "OFF",
                                                status_word);
                            }
                            last_driver_status_word = status_word;
                        }
                        break;
                    }
                }
            }
        }
    }

    // Таймаут связи только в рабочем режиме (init сам обновляет дедлайн эхо-ответами)
    if (!encoder_fault_latched &&
        current_state == DriveState::RUN_ACTIVE &&
        link_deadline_ms != 0 &&
        now > link_deadline_ms)
    {
        if (modbus_link_ok) {
            gcs().send_text(MAV_SEVERITY_WARNING,
                            "CL57R: Modbus link lost, re-initializing...");
            modbus_link_ok = false;
        }
        current_state = DriveState::INIT_ENABLE;
        response_received = false;
        last_driver_error_code = 0;
        last_driver_status_word = 0;
        run_status_div = 0;
        expecting_status_read = false;
        pending_encoder_read = false;
        last_sent_target_pulses = 0;
        have_sent_target = false;
        have_actual_position = false;
        need_position_sync = false;
        have_valid_actual = false;
        run_phase_write = true;
        link_deadline_ms = now + MODBUS_LINK_TIMEOUT_MS;
    }

    // --- БЛОК ОТПРАВКИ КОМАНД ПО ТАЙМЕРУ (уставка 20 Гц, статус 1 Гц) ---
    if ((now - _last_send_ms) < POSITION_SEND_INTERVAL_MS)
    {
        return;
    }
    if (_uart->txspace() < 22)
    {
        _last_send_ms = now;
        return;
    }
    _last_send_ms = now;
    if (link_deadline_ms == 0) {
        link_deadline_ms = now + MODBUS_LINK_TIMEOUT_MS;
    }
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
            current_state = DriveState::INIT_TRACK_ERR;
            break;
        case DriveState::INIT_TRACK_ERR:
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
            current_state = DriveState::INIT_ABS_MODE;
            break;
        case DriveState::INIT_ABS_MODE:
            current_state = DriveState::INIT_TRIG_MODE;
            break;
        case DriveState::INIT_TRIG_MODE:
            current_state = DriveState::RUN_ACTIVE;
            last_sent_target_pulses = 0;
            have_sent_target = false;
            have_actual_position = false;
            need_position_sync = true;
            have_valid_actual = false;
            run_status_div = 0;
            run_phase_write = true;
            pending_encoder_read = false;
            modbus_note_link_rx(now, link_deadline_ms);
            if (!modbus_link_ok) {
                gcs().send_text(MAV_SEVERITY_INFO,
                                "CL57R: Modbus link restored, driver ready");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "CL57R: Modbus Driver READY.");
            }
            modbus_link_ok = true;
            break;
        case DriveState::TRACK_CLEAR:
            current_state = DriveState::RUN_ACTIVE;
            break;
        case DriveState::RUN_ACTIVE:
            break;
        case DriveState::FAULT_RELEASE:
            current_state = DriveState::FAULT_LATCHED;
            break;
        case DriveState::FAULT_LATCHED:
            break;
        }
    }

    // Отправка пакетов на основе текущего состояния
    switch (current_state)
    {
    case DriveState::INIT_ENABLE:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_MOTOR_ENABLE, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        break;

    case DriveState::FAULT_RELEASE:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_MOTOR_ENABLE, 0x0000, tx_packet);
        _uart->write(tx_packet, 8);
        have_sent_target = false;
        last_sent_target_pulses = 0;
        break;

    case DriveState::FAULT_LATCHED:
        modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
        _uart->write(tx_packet, 8);
        break;

    case DriveState::TRACK_CLEAR:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_AUX_CONTROL, AUX_ALARM_CLEAR, tx_packet);
        _uart->write(tx_packet, 8);
        need_position_sync = true;
        break;

    case DriveState::INIT_CLEAR_ALARM:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_AUX_CONTROL, AUX_ALARM_CLEAR, tx_packet);
        _uart->write(tx_packet, 8);
        break;

    case DriveState::INIT_SUBDIVISION:
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0023, 4000, tx_packet);
        _uart->write(tx_packet, 8);
        break;

    case DriveState::INIT_TRACK_ERR:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_TRACK_ERR, CL57R_TRACK_ERR_DEFAULT, tx_packet);
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
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0031, (uint16_t)accel_ms.get(), tx_packet);
        _uart->write(tx_packet, 8);
        break;

    case DriveState::INIT_DECEL:
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0032, (uint16_t)decel_ms.get(), tx_packet);
        _uart->write(tx_packet, 8);
        break;

    case DriveState::INIT_ABS_MODE:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_POS_MODE, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        break;

    case DriveState::INIT_TRIG_MODE:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_TRIGGER_MODE, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        break;

         case DriveState::RUN_ACTIVE: {
            if (encoder_fault_latched) {
                current_state = DriveState::FAULT_LATCHED;
                break;
            }

            if (pending_track_clear) {
                pending_track_clear = false;
                current_state = DriveState::TRACK_CLEAR;
                modbus_create_write_packet((uint8_t)slave_id.get(), REG_AUX_CONTROL, AUX_ALARM_CLEAR, tx_packet);
                _uart->write(tx_packet, 8);
                break;
            }

            if (run_phase_write) {
                run_phase_write = false;

                if (!have_actual_position) {
                    run_phase_write = true;
                    expecting_status_read = false;
                    pending_encoder_read = true;
                    modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
                    _uart->write(tx_packet, 8);
                    break;
                }

                if (now < tracking_pause_until_ms) {
                    debug_target_pulses = debug_actual_pulses;
                    break;
                }

                float clean_steering = steering_out;
                if (clean_steering > 1.0f)  clean_steering = 1.0f;
                if (clean_steering < -1.0f) clean_steering = -1.0f;
                if (invert.get() != 0) {
                    clean_steering = -clean_steering;
                }

                static bool was_returning = true;
                const bool returning = fabsf(clean_steering) < STICK_CENTER_THRESHOLD;
                if (was_returning && !returning) {
                    need_position_sync = true;
                }
                was_returning = returning;

                if (fabsf(clean_steering) > STICK_CENTER_THRESHOLD &&
                    now < tracking_pause_until_ms) {
                    tracking_pause_until_ms = 0;
                    need_position_sync = true;
                }

                const int32_t max_pulses = travel_limit_pulses();
                const int32_t deadband = pos_db.get();

                // Если стик в мёртвой зоне центра — форсируем desired=0.
                // Это обеспечивает возврат в ноль при отпускании стика даже при неточном triм.
                const int32_t desired = returning
                    ? 0
                    : clamp_int32((int32_t)(clean_steering * (float)max_pulses),
                                  -max_pulses, max_pulses);

                // Шаг за 1 такт при MAX_SPD.
                const int32_t active_limit = speed_move_limit_pulses((uint16_t)max_speed.get(),
                                                                     POSITION_SEND_INTERVAL_MS * 2);

                bool just_synced = false;
                int32_t commanded;

                if (need_position_sync) {
                    last_sent_target_pulses = debug_actual_pulses;
                    need_position_sync = false;
                    just_synced = true;
                }

                const bool past_limit = abs_int32(debug_actual_pulses) > max_pulses;

                if (past_limit) {
                    // Мотор за лимитом. Если стик ведёт к центру — следовать ему.
                    const bool going_back = (debug_actual_pulses > 0)
                        ? (desired < debug_actual_pulses)
                        : (desired > debug_actual_pulses);
                    if (going_back) {
                        commanded = desired;
                    } else if (debug_actual_pulses > 0) {
                        commanded = max_pulses - active_limit;
                    } else {
                        commanded = -max_pulses + active_limit;
                    }
                } else if (returning) {
                    // Возврат в центр: плавно двигать цель к нулю со скоростью мотора.
                    // Это предотвращает перелёт нуля: мотор гонится за движущейся целью
                    // и тормозит по пути, а не тормозит в одной точке.
                    const int32_t step = (ret_slew.get() > 0)
                        ? MIN((int32_t)ret_slew.get(), active_limit)
                        : active_limit;
                    commanded = step_toward(last_sent_target_pulses, 0, step);
                } else {
                    // Прямое управление: стик = абсолютная позиция.
                    commanded = desired;
                }

                commanded = clamp_int32(commanded, -max_pulses, max_pulses);

                static bool soft_limit_warned = false;
                if (abs_int32(debug_actual_pulses) > max_pulses) {
                    const int32_t overshoot = abs_int32(debug_actual_pulses) - max_pulses;
                    if (!soft_limit_warned && overshoot > active_limit) {
                        gcs().send_text(MAV_SEVERITY_WARNING,
                                        "CL57R: overshoot %ld imp, correcting",
                                        (long)overshoot);
                        soft_limit_warned = true;
                    } else if (overshoot <= deadband) {
                        soft_limit_warned = false;
                    }
                } else {
                    soft_limit_warned = false;
                }

                debug_target_pulses = commanded;

                const bool past_limit_now = abs_int32(debug_actual_pulses) > max_pulses;
                // near_limit_now: дополнительный триггер только при активном рулении.
                const bool near_limit_now = !returning &&
                                            abs_int32(debug_actual_pulses) >
                                            max_pulses - active_limit * 4;
                const bool should_send = just_synced ||
                                         !have_sent_target ||
                                         commanded != last_sent_target_pulses ||
                                         past_limit_now ||
                                         near_limit_now ||
                                         (!returning &&
                                          abs_int32(debug_actual_pulses - commanded) > deadband);

                if (should_send) {
                    uint16_t values[3];
                    values[0] = (uint16_t)((commanded >> 16) & 0xFFFF);
                    values[1] = (uint16_t)(commanded & 0xFFFF);
                    values[2] = MOTION_START_ABS;

                    modbus_create_write_multiple_packet((uint8_t)slave_id.get(), 0x0034, 3, values, tx_packet);
                    _uart->write(tx_packet, 15);
                    last_sent_target_pulses = commanded;
                    have_sent_target = true;
                }
            } else {
                run_phase_write = true;
                run_status_div++;
                if (run_status_div >= STATUS_READ_EVERY_N_READS) {
                    run_status_div = 0;
                    expecting_status_read = true;
                    pending_encoder_read = false;
                    modbus_create_read_packet((uint8_t)slave_id.get(), REG_STATUS_WORD, 2, tx_packet);
                } else {
                    expecting_status_read = false;
                    pending_encoder_read = true;
                    modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
                }
                _uart->write(tx_packet, 8);
            }
            break;
        }
    }
}