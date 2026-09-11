#include "AP_ModbusSteering.h"

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>

extern "C" {
    void modbus_create_write_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t value, uint8_t *out_buffer);
    void modbus_create_write_multiple_packet(uint8_t slave_id, uint16_t start_reg, uint16_t reg_count, const uint16_t *reg_values, uint8_t *out_buffer);
    void modbus_create_read_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count, uint8_t *out_buffer);
    uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
}

extern const AP_HAL::HAL &hal;

namespace {
constexpr int32_t CL57R_STEPS_PER_REV = 4000;
constexpr uint16_t REG_ENCODER_POS = 0x0007;
constexpr uint16_t REG_TARGET_POS = 0x0034;
constexpr uint16_t REG_AUX_CONTROL = 0x0037;
constexpr uint16_t REG_MOTOR_ENABLE = 0x0038;
constexpr uint16_t REG_POS_MODE = 0x003A;
constexpr uint16_t AUX_ALARM_CLEAR = 0x0004;
constexpr uint16_t MOTION_START_ABS = 0x0003;
constexpr uint16_t SUBDIVISION_PPR = 4000;
constexpr uint16_t ACCEL_DEFAULT = 200;
constexpr uint16_t DECEL_DEFAULT = 200;
constexpr uint32_t SEND_INTERVAL_MS = 50;
constexpr uint8_t INIT_SKIP_ATTEMPTS = 40;
}

const AP_Param::GroupInfo AP_ModbusSteering::var_info[] = {
    // @Param: SLAVE_ID
    // @DisplayName: Modbus Slave ID
    // @Description: CL57R Modbus slave address
    // @Range: 1 247
    // @User: Standard
    AP_GROUPINFO("SLAVE_ID", 1, AP_ModbusSteering, slave_id, 1),

    // @Param: REG_ADDR
    // @DisplayName: Base Register Address
    // @Description: Position register 0x0034 as decimal (52)
    // @User: Advanced
    AP_GROUPINFO("REG_ADDR", 2, AP_ModbusSteering, reg_address, 52),

    // @Param: MAX_STEPS
    // @DisplayName: Maximum Steering Steps
    // @Description: Pulses at full stick (+/-1) when OUT_REV=0. Otherwise OUT_REV*RATIO*4000/2.
    // @User: Standard
    AP_GROUPINFO("MAX_STEPS", 3, AP_ModbusSteering, max_steps, 200000),

    // @Param: START_SPD
    // @DisplayName: Start Speed
    // @Description: CL57R start speed in RPM (register 0x0030), written once at init
    // @Units: RPM
    // @User: Standard
    AP_GROUPINFO("START_SPD", 4, AP_ModbusSteering, start_speed, 15),

    // @Param: MAX_SPD
    // @DisplayName: Maximum Speed
    // @Description: CL57R max speed in RPM (register 0x0033), written once at init
    // @Units: RPM
    // @User: Standard
    AP_GROUPINFO("MAX_SPD", 5, AP_ModbusSteering, max_speed, 120),

    // @Param: POS_DB
    // @DisplayName: Position Deadband
    // @Description: Skip a new position write if the stick target changed by less than this (pulses). 0 = always write.
    // @Units: pulses
    // @Range: 0 50000
    // @User: Standard
    AP_GROUPINFO("POS_DB", 6, AP_ModbusSteering, pos_db, 500),

    // @Param: RET_SLEW
    // @DisplayName: Return slew (unused)
    // @Description: Unused. Kept so existing parameter storage indices stay valid.
    // @User: Advanced
    AP_GROUPINFO("RET_SLEW", 7, AP_ModbusSteering, ret_slew, 0),

    // @Param: OUT_REV
    // @DisplayName: Rudder Lock-to-Lock Turns
    // @Description: Output-shaft turns lock-to-lock. If >0, travel limit is OUT_REV*RATIO*4000/2 pulses.
    // @Units: rev
    // @Range: 0 20
    // @User: Standard
    AP_GROUPINFO("OUT_REV", 8, AP_ModbusSteering, out_rev, 4),

    // @Param: RATIO
    // @DisplayName: Gearbox Ratio
    // @Description: Gearbox ratio (motor rev per output rev)
    // @Range: 1 200
    // @User: Standard
    AP_GROUPINFO("RATIO", 9, AP_ModbusSteering, ratio, 25),

    AP_GROUPEND
};

AP_ModbusSteering::AP_ModbusSteering()
{
    AP_Param::setup_object_defaults(this, var_info);
}

int32_t AP_ModbusSteering::travel_limit_pulses() const
{
    if (out_rev.get() > 0 && ratio.get() > 0) {
        return (int32_t)out_rev.get() * ratio.get() * CL57R_STEPS_PER_REV / 2;
    }
    return max_steps.get();
}

uint8_t AP_ModbusSteering::rtu_frame_len(const uint8_t *buf, uint8_t avail) const
{
    if (avail < 2) {
        return 0;
    }
    const uint8_t fn = buf[1];
    if (fn == 0x06 || fn == 0x10) {
        return 8;
    }
    if (fn == 0x03) {
        if (avail < 3) {
            return 0;
        }
        return (uint8_t)(5 + buf[2]);
    }
    if ((fn & 0x80) != 0) {
        return 5;
    }
    return 0;
}

void AP_ModbusSteering::init(AP_SerialManager &serial_manager)
{
    _uart = serial_manager.find_serial((AP_SerialManager::SerialProtocol)101, 0);
    if (_uart != nullptr) {
        _uart->begin(115200);
    }
}

void AP_ModbusSteering::consume_rx()
{
    uint32_t available = _uart->available();
    while (available > 0) {
        if (_rx_len >= sizeof(_rx_buf)) {
            for (uint8_t i = 1; i < _rx_len; i++) {
                _rx_buf[i - 1] = _rx_buf[i];
            }
            _rx_len--;
        }
        _rx_buf[_rx_len++] = _uart->read();
        available--;
        _last_rx_ms = AP_HAL::millis();
    }

    uint16_t offset = 0;
    while ((uint16_t)_rx_len - offset >= 5) {
        if (_rx_buf[offset] != (uint8_t)slave_id.get()) {
            offset++;
            continue;
        }
        const uint8_t frame_len = rtu_frame_len(&_rx_buf[offset], (uint8_t)(_rx_len - offset));
        if (frame_len == 0) {
            offset++;
            continue;
        }
        if ((uint16_t)(_rx_len - offset) < frame_len) {
            break;
        }
        const uint16_t received_crc = (_rx_buf[offset + frame_len - 1] << 8) | _rx_buf[offset + frame_len - 2];
        if (modbus_crc16(&_rx_buf[offset], frame_len - 2) != received_crc) {
            offset++;
            continue;
        }

        const uint8_t fn = _rx_buf[offset + 1];
        if (fn == 0x06 && _state < DriveState::RUN_WRITE) {
            _got_echo = true;
        } else if (fn == 0x03 && frame_len >= 9 && _rx_buf[offset + 2] == 0x04) {
            const uint16_t high_word = (_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4];
            const uint16_t low_word = (_rx_buf[offset + 5] << 8) | _rx_buf[offset + 6];
            _actual_pulses = (int32_t)(((uint32_t)high_word << 16) | low_word);
        }

        offset = (uint16_t)(offset + frame_len);
    }

    if (offset > 0) {
        const uint8_t remain = (uint8_t)(_rx_len - offset);
        for (uint8_t i = 0; i < remain; i++) {
            _rx_buf[i] = _rx_buf[offset + i];
        }
        _rx_len = remain;
    }
}

void AP_ModbusSteering::advance_init()
{
    _got_echo = false;
    _init_attempts = 0;
    switch (_state) {
    case DriveState::INIT_ENABLE:
        _state = DriveState::INIT_CLEAR_ALARM;
        break;
    case DriveState::INIT_CLEAR_ALARM:
        _state = DriveState::INIT_SUBDIVISION;
        break;
    case DriveState::INIT_SUBDIVISION:
        _state = DriveState::INIT_START_SPD;
        break;
    case DriveState::INIT_START_SPD:
        _state = DriveState::INIT_MAX_SPD;
        break;
    case DriveState::INIT_MAX_SPD:
        _state = DriveState::INIT_ACCEL;
        break;
    case DriveState::INIT_ACCEL:
        _state = DriveState::INIT_DECEL;
        break;
    case DriveState::INIT_DECEL:
        _state = DriveState::INIT_ABS_MODE;
        break;
    case DriveState::INIT_ABS_MODE:
        _state = DriveState::RUN_WRITE;
        _last_rx_ms = AP_HAL::millis();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: Modbus Driver READY.");
        break;
    default:
        break;
    }
}

void AP_ModbusSteering::update(float steering_out)
{
    if (_uart == nullptr || !_uart->is_initialized()) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    consume_rx();

    const int32_t max_pulses = travel_limit_pulses();
    const int32_t stick_pulses = constrain_int32((int32_t)(steering_out * (float)max_pulses),
                                                 -max_pulses, max_pulses);
    if (now - _last_vect_ms >= 200) {
        _last_vect_ms = now;
        gcs().send_debug_vect("STEER",
                              (float)_actual_pulses,
                              (float)stick_pulses,
                              (float)_last_target);
    }

    if ((now - _last_send_ms) < SEND_INTERVAL_MS) {
        return;
    }
    if (_uart->txspace() < 22) {
        return;
    }
    _last_send_ms = now;

    if (_state < DriveState::RUN_WRITE) {
        if (_got_echo) {
            advance_init();
        } else {
            if (_init_attempts == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: init step %d", (int)_state);
            }
            _init_attempts++;
            if (_init_attempts >= INIT_SKIP_ATTEMPTS) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: init step %d no echo, continuing",
                              (int)_state);
                advance_init();
            }
        }
    } else if (_last_rx_ms != 0 && (now - _last_rx_ms) > 2000) {
        _last_rx_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: Modbus Timeout, continuing");
    }

    uint8_t tx_packet[16];
    switch (_state) {
    case DriveState::INIT_ENABLE:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_MOTOR_ENABLE, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        break;
    case DriveState::INIT_CLEAR_ALARM:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_AUX_CONTROL, AUX_ALARM_CLEAR, tx_packet);
        _uart->write(tx_packet, 8);
        break;
    case DriveState::INIT_SUBDIVISION:
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0023, SUBDIVISION_PPR, tx_packet);
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
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0031, ACCEL_DEFAULT, tx_packet);
        _uart->write(tx_packet, 8);
        break;
    case DriveState::INIT_DECEL:
        modbus_create_write_packet((uint8_t)slave_id.get(), 0x0032, DECEL_DEFAULT, tx_packet);
        _uart->write(tx_packet, 8);
        break;
    case DriveState::INIT_ABS_MODE:
        modbus_create_write_packet((uint8_t)slave_id.get(), REG_POS_MODE, 0x0001, tx_packet);
        _uart->write(tx_packet, 8);
        break;
    case DriveState::RUN_WRITE: {
        const int32_t target = stick_pulses;
        const int32_t deadband = pos_db.get();
        const int32_t delta = (target > _last_target) ? (target - _last_target) : (_last_target - target);
        const bool send_pos = !_have_target || (delta > deadband);
        if (send_pos) {
            uint16_t values[3];
            values[0] = (uint16_t)((target >> 16) & 0xFFFF);
            values[1] = (uint16_t)(target & 0xFFFF);
            values[2] = MOTION_START_ABS;
            modbus_create_write_multiple_packet((uint8_t)slave_id.get(), REG_TARGET_POS, 3, values, tx_packet);
            _uart->write(tx_packet, 15);
            _last_target = target;
            _have_target = true;
        } else {
            modbus_create_write_packet((uint8_t)slave_id.get(), REG_MOTOR_ENABLE, 0x0001, tx_packet);
            _uart->write(tx_packet, 8);
        }
        _state = DriveState::RUN_READ;
        break;
    }
    case DriveState::RUN_READ:
        modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
        _uart->write(tx_packet, 8);
        _state = DriveState::RUN_WRITE;
        break;
    }
}
