#include "AP_ModbusSteering.h"

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

extern "C" {
    void modbus_create_write_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t value, uint8_t *out_buffer);
    void modbus_create_write_multiple_packet(uint8_t slave_id, uint16_t start_reg, uint16_t reg_count, const uint16_t *reg_values, uint8_t *out_buffer);
    void modbus_create_read_packet(uint8_t slave_id, uint16_t reg_addr, uint16_t reg_count, uint8_t *out_buffer);
    uint16_t modbus_crc16(const uint8_t *buf, uint16_t len);
}

extern const AP_HAL::HAL &hal;

namespace {
constexpr int32_t CL57R_STEPS_PER_REV = 4000;
constexpr uint16_t REG_STATUS = 0x0003;
constexpr uint16_t REG_ENCODER_POS = 0x0007;
constexpr uint16_t REG_HOME_METHOD = 0x0040;
constexpr uint16_t REG_HOME_SPD = 0x0041;
constexpr uint16_t REG_HOME_CRAWL = 0x0042;
constexpr uint16_t REG_HOME_ACCEL = 0x0043;
constexpr uint16_t REG_MAX_SPD = 0x0033;
constexpr uint16_t REG_TARGET_POS = 0x0034;
constexpr uint16_t REG_MOTION = 0x0036;
constexpr uint16_t REG_AUX_CONTROL = 0x0037;
constexpr uint16_t REG_MOTOR_ENABLE = 0x0038;
constexpr uint16_t REG_POS_MODE = 0x003A;
constexpr uint16_t AUX_ALARM_CLEAR = 0x0004;
constexpr uint16_t AUX_POS_ZERO = 0x0008;
constexpr uint16_t MOTION_START_ABS_IRQ = 0x0007;
constexpr uint16_t MOTION_HOME = 0x0010;
constexpr uint16_t STATUS_HOME_DONE = (1U << 1);
constexpr uint16_t STATUS_RUNNING = (1U << 2);
constexpr uint16_t SUBDIVISION_PPR = 4000;
constexpr uint16_t ACCEL_DEFAULT = 200;
constexpr uint16_t DECEL_DEFAULT = 200;
constexpr uint32_t SEND_INTERVAL_MS = 50;
constexpr uint32_t BUTTON_LOCKOUT_MS = 300;
constexpr uint32_t HOME_TIMEOUT_MS = 90000;
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
    // @DisplayName: Armed run speed
    // @Description: CL57R max speed in RPM (register 0x0033) used after calibration and while armed
    // @Units: RPM
    // @Range: 1 3000
    // @User: Standard
    AP_GROUPINFO("MAX_SPD", 5, AP_ModbusSteering, max_speed, 1300),

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

    // @Param: RST_CH
    // @DisplayName: Alarm reset RC channel
    // @Description: RC channel that clears the CL57R alarm on a rising edge (PWM above 1800). 0 disables the button.
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("RST_CH", 10, AP_ModbusSteering, rst_ch, 0),

    // @Param: HOME_CH
    // @DisplayName: Homing RC channel
    // @Description: RC channel that starts CL57R homing on a rising edge (PWM above 1800). Clears the alarm first, then homes to the limit and shifts zero to mid-travel. Ignored while armed. 0 disables the button.
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("HOME_CH", 11, AP_ModbusSteering, home_ch, 0),

    // @Param: HOME_MTH
    // @DisplayName: CL57R home method
    // @Description: Native CL57R homing method. 17 searches the negative limit (X2 N-OT). 18 searches the positive limit (X1 P-OT).
    // @Values: 17:NegativeLimit,18:PositiveLimit
    // @Range: 17 18
    // @User: Standard
    AP_GROUPINFO("HOME_MTH", 12, AP_ModbusSteering, home_mth, 17),

    // @Param: HOME_SPD
    // @DisplayName: Homing speed
    // @Description: CL57R speed in RPM during calibration (registers 0x0041 and 0x0033 while homing)
    // @Units: RPM
    // @Range: 1 3000
    // @User: Standard
    AP_GROUPINFO("HOME_SPD", 13, AP_ModbusSteering, home_speed, 900),

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

bool AP_ModbusSteering::in_run() const
{
    return _state == DriveState::RUN_WRITE || _state == DriveState::RUN_READ;
}

bool AP_ModbusSteering::in_home() const
{
    return _state >= DriveState::HOME_CLEAR_ALARM;
}

bool AP_ModbusSteering::home_prep_wait_echo() const
{
    return (_state >= DriveState::HOME_CLEAR_ALARM && _state <= DriveState::HOME_START) ||
           _state == DriveState::HOME_ZERO;
}

uint16_t AP_ModbusSteering::home_method_reg() const
{
    const int8_t method = home_mth.get();
    if (method == 18) {
        return 18;
    }
    return 17;
}

uint16_t AP_ModbusSteering::run_speed_rpm() const
{
    const int16_t rpm = max_speed.get();
    if (rpm < 1) {
        return 1300;
    }
    return (uint16_t)rpm;
}

uint16_t AP_ModbusSteering::calib_speed_rpm() const
{
    const int16_t rpm = home_speed.get();
    if (rpm < 1) {
        return 900;
    }
    return (uint16_t)rpm;
}

int32_t AP_ModbusSteering::center_target_pulses() const
{
    const int32_t limit = travel_limit_pulses();
    if (home_method_reg() == 18) {
        return -limit;
    }
    return limit;
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

void AP_ModbusSteering::send_u16(uint16_t reg, uint16_t value)
{
    uint8_t tx_packet[8];
    modbus_create_write_packet((uint8_t)slave_id.get(), reg, value, tx_packet);
    _uart->write(tx_packet, 8);
}

void AP_ModbusSteering::send_abs_move(int32_t target)
{
    uint8_t tx_packet[16];
    uint16_t values[3];
    values[0] = (uint16_t)((target >> 16) & 0xFFFF);
    values[1] = (uint16_t)(target & 0xFFFF);
    values[2] = MOTION_START_ABS_IRQ;
    modbus_create_write_multiple_packet((uint8_t)slave_id.get(), REG_TARGET_POS, 3, values, tx_packet);
    _uart->write(tx_packet, 15);
    _last_target = target;
    _have_target = true;
}

bool AP_ModbusSteering::rc_rising_edge(int8_t ch, bool &was_high) const
{
    if (ch < 1 || ch > 16) {
        return false;
    }
    uint16_t pwm = 0;
    if (!rc().get_pwm((uint8_t)ch, pwm)) {
        was_high = false;
        return false;
    }
    if (pwm > RC_Channel::AUX_SWITCH_PWM_TRIGGER_HIGH) {
        const bool edge = !was_high;
        was_high = true;
        return edge;
    }
    if (pwm < RC_Channel::AUX_SWITCH_PWM_TRIGGER_LOW) {
        was_high = false;
    }
    return false;
}

void AP_ModbusSteering::request_alarm_clear()
{
    if (in_home()) {
        return;
    }
    _alarm_clear_pending = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: alarm clear (RC)");
}

void AP_ModbusSteering::start_home()
{
    if (hal.util->get_soft_armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: home ignored, armed");
        return;
    }
    if (in_home()) {
        return;
    }
    if (_state < DriveState::RUN_WRITE) {
        _home_pending = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home queued");
        return;
    }
    _alarm_clear_pending = false;
    _homed = false;
    _got_echo = false;
    _init_attempts = 0;
    _got_status = false;
    _saw_home_run = false;
    _state = DriveState::HOME_CLEAR_ALARM;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home start (RC)");
}

void AP_ModbusSteering::poll_rc_buttons()
{
    const uint32_t now = AP_HAL::millis();
    const bool rst_edge = rc_rising_edge(rst_ch.get(), _rst_was_high);
    const bool home_edge = rc_rising_edge(home_ch.get(), _home_was_high);
    if (now - _last_button_ms < BUTTON_LOCKOUT_MS) {
        return;
    }
    if (home_edge) {
        _last_button_ms = now;
        start_home();
        return;
    }
    if (rst_edge) {
        _last_button_ms = now;
        request_alarm_clear();
    }
}

void AP_ModbusSteering::finish_home()
{
    _state = DriveState::RUN_WRITE;
    _have_target = false;
    _last_target = 0;
    _homed = true;
    _pending_run_spd = true;
    _rx_expect = RxExpect::NONE;
    _last_rx_ms = AP_HAL::millis();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: calibrated");
}

void AP_ModbusSteering::abort_home(const char *reason)
{
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: %s", reason);
    _state = DriveState::RUN_WRITE;
    _have_target = false;
    _rx_expect = RxExpect::NONE;
    _last_rx_ms = AP_HAL::millis();
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
        if (fn == 0x06) {
            _got_echo = true;
        } else if (fn == 0x03 && frame_len >= 7) {
            const uint8_t byte_count = _rx_buf[offset + 2];
            if (_rx_expect == RxExpect::STATUS && byte_count >= 2) {
                _status_word = ((uint16_t)_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4];
                _got_status = true;
                if ((_status_word & STATUS_RUNNING) != 0) {
                    _saw_home_run = true;
                }
            } else if (_rx_expect == RxExpect::ENCODER && byte_count == 0x04 && frame_len >= 9) {
                const uint16_t high_word = (_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4];
                const uint16_t low_word = (_rx_buf[offset + 5] << 8) | _rx_buf[offset + 6];
                _actual_pulses = (int32_t)(((uint32_t)high_word << 16) | low_word);
            }
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
        if (_home_pending) {
            _home_pending = false;
            start_home();
        }
        break;
    default:
        break;
    }
}

void AP_ModbusSteering::advance_home()
{
    _got_echo = false;
    _init_attempts = 0;
    switch (_state) {
    case DriveState::HOME_CLEAR_ALARM:
        _state = DriveState::HOME_SET_METHOD;
        break;
    case DriveState::HOME_SET_METHOD:
        _state = DriveState::HOME_SET_SPD;
        break;
    case DriveState::HOME_SET_SPD:
        _state = DriveState::HOME_SET_RUN_SPD;
        break;
    case DriveState::HOME_SET_RUN_SPD:
        _state = DriveState::HOME_SET_CRAWL;
        break;
    case DriveState::HOME_SET_CRAWL:
        _state = DriveState::HOME_SET_ACCEL;
        break;
    case DriveState::HOME_SET_ACCEL:
        _state = DriveState::HOME_START;
        break;
    case DriveState::HOME_START:
        _state = DriveState::HOME_WAIT;
        _home_start_ms = AP_HAL::millis();
        _got_status = false;
        _saw_home_run = false;
        break;
    case DriveState::HOME_ZERO:
        finish_home();
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
    poll_rc_buttons();

    const bool armed = hal.util->get_soft_armed();
    if (armed && !_was_armed) {
        _pending_run_spd = true;
    }
    _was_armed = armed;

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

    if (_state == DriveState::HOME_WAIT) {
        const bool home_bit = (_got_status && (_status_word & STATUS_HOME_DONE) != 0);
        const bool running = (_got_status && (_status_word & STATUS_RUNNING) != 0);
        if (now - _home_start_ms > HOME_TIMEOUT_MS) {
            abort_home("home timeout");
        } else if (home_bit && !running &&
                   (_saw_home_run || (now - _home_start_ms >= 2000))) {
            _center_target = center_target_pulses();
            _state = DriveState::HOME_MOVE_CENTER;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: centering after home");
        }
    } else if (_state == DriveState::HOME_WAIT_CENTER) {
        const int32_t err = (_actual_pulses > _center_target) ?
                            (_actual_pulses - _center_target) : (_center_target - _actual_pulses);
        int32_t arrive = pos_db.get();
        if (arrive < 1000) {
            arrive = 1000;
        }
        if (now - _home_start_ms > HOME_TIMEOUT_MS) {
            abort_home("center timeout");
        } else if (err <= arrive) {
            _state = DriveState::HOME_ZERO;
            _got_echo = false;
            _init_attempts = 0;
        }
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
    } else if (in_run()) {
        if (_last_rx_ms != 0 && (now - _last_rx_ms) > 2000) {
            _last_rx_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: Modbus Timeout, continuing");
        }
    } else if (home_prep_wait_echo()) {
        if (_got_echo) {
            advance_home();
        } else {
            _init_attempts++;
            if (_init_attempts >= INIT_SKIP_ATTEMPTS) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: home step %d no echo, continuing",
                              (int)_state);
                advance_home();
            }
        }
    }

    uint8_t tx_packet[16];
    switch (_state) {
    case DriveState::INIT_ENABLE:
        send_u16(REG_MOTOR_ENABLE, 0x0001);
        break;
    case DriveState::INIT_CLEAR_ALARM:
        send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
        break;
    case DriveState::INIT_SUBDIVISION:
        send_u16(0x0023, SUBDIVISION_PPR);
        break;
    case DriveState::INIT_START_SPD:
        send_u16(0x0030, (uint16_t)start_speed.get());
        break;
    case DriveState::INIT_MAX_SPD:
        send_u16(REG_MAX_SPD, run_speed_rpm());
        break;
    case DriveState::INIT_ACCEL:
        send_u16(0x0031, ACCEL_DEFAULT);
        break;
    case DriveState::INIT_DECEL:
        send_u16(0x0032, DECEL_DEFAULT);
        break;
    case DriveState::INIT_ABS_MODE:
        send_u16(REG_POS_MODE, 0x0001);
        break;
    case DriveState::RUN_WRITE: {
        if (_alarm_clear_pending) {
            send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
            _alarm_clear_pending = false;
            _state = DriveState::RUN_READ;
            break;
        }
        if (_pending_run_spd) {
            send_u16(REG_MAX_SPD, run_speed_rpm());
            _pending_run_spd = false;
            _state = DriveState::RUN_READ;
            break;
        }
        const int32_t target = stick_pulses;
        const int32_t deadband = pos_db.get();
        const int32_t delta = (target > _last_target) ? (target - _last_target) : (_last_target - target);
        const bool send_pos = !_have_target || (delta > deadband);
        if (send_pos) {
            send_abs_move(target);
        } else {
            send_u16(REG_MOTOR_ENABLE, 0x0001);
        }
        _state = DriveState::RUN_READ;
        break;
    }
    case DriveState::RUN_READ:
        if (_read_status_next) {
            _rx_expect = RxExpect::STATUS;
            modbus_create_read_packet((uint8_t)slave_id.get(), REG_STATUS, 2, tx_packet);
        } else {
            _rx_expect = RxExpect::ENCODER;
            modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
        }
        _read_status_next = !_read_status_next;
        _uart->write(tx_packet, 8);
        _state = DriveState::RUN_WRITE;
        break;
    case DriveState::HOME_CLEAR_ALARM:
        send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
        break;
    case DriveState::HOME_SET_METHOD:
        send_u16(REG_HOME_METHOD, home_method_reg());
        break;
    case DriveState::HOME_SET_SPD:
        send_u16(REG_HOME_SPD, calib_speed_rpm());
        break;
    case DriveState::HOME_SET_RUN_SPD:
        send_u16(REG_MAX_SPD, calib_speed_rpm());
        break;
    case DriveState::HOME_SET_CRAWL:
        send_u16(REG_HOME_CRAWL, (uint16_t)start_speed.get());
        break;
    case DriveState::HOME_SET_ACCEL:
        send_u16(REG_HOME_ACCEL, ACCEL_DEFAULT);
        break;
    case DriveState::HOME_START:
        send_u16(REG_MOTION, MOTION_HOME);
        break;
    case DriveState::HOME_WAIT:
        _rx_expect = RxExpect::STATUS;
        modbus_create_read_packet((uint8_t)slave_id.get(), REG_STATUS, 2, tx_packet);
        _uart->write(tx_packet, 8);
        break;
    case DriveState::HOME_MOVE_CENTER:
        send_abs_move(_center_target);
        _home_start_ms = now;
        _got_status = false;
        _state = DriveState::HOME_WAIT_CENTER;
        break;
    case DriveState::HOME_WAIT_CENTER:
        _rx_expect = RxExpect::ENCODER;
        modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
        _uart->write(tx_packet, 8);
        break;
    case DriveState::HOME_ZERO:
        send_u16(REG_AUX_CONTROL, AUX_POS_ZERO);
        break;
    }
}
