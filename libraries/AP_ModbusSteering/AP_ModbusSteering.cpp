#include "AP_ModbusSteering.h"
#include "modbus_protocol.h"

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL &hal;

namespace {
constexpr int32_t STEPS_PER_REV = 4000;
constexpr uint16_t REG_STATUS = 0x0003;
constexpr uint16_t REG_DI = 0x0005;
constexpr uint16_t REG_ENCODER = 0x0007;
constexpr uint16_t REG_DI_POLARITY = 0x0010;
constexpr uint16_t REG_X1_FN = 0x0014;
constexpr uint16_t REG_X2_FN = 0x0015;
constexpr uint16_t REG_START_SPD = 0x0030;
constexpr uint16_t REG_ACCEL = 0x0031;
constexpr uint16_t REG_DECEL = 0x0032;
constexpr uint16_t REG_MAX_SPD = 0x0033;
constexpr uint16_t REG_TARGET_POS = 0x0034;
constexpr uint16_t REG_MOTION = 0x0036;
constexpr uint16_t REG_AUX = 0x0037;
constexpr uint16_t REG_ENABLE = 0x0038;
constexpr uint16_t REG_HOME_METHOD = 0x0040;
constexpr uint16_t REG_HOME_SPD = 0x0041;
constexpr uint16_t REG_HOME_CRAWL = 0x0042;
constexpr uint16_t REG_HOME_ACCEL = 0x0043;

constexpr uint16_t FN_NONE = 0;
constexpr uint16_t AUX_ALARM_CLEAR = 0x0004;
constexpr uint16_t AUX_POS_ZERO = 0x0008;
constexpr uint16_t MOTION_REL = 0x0001;
constexpr uint16_t MOTION_ABS = 0x0003;
constexpr uint16_t MOTION_ABS_IRQ = 0x0007; // abs start + interrupt current move
constexpr uint16_t MOTION_SPEED = 0x0008;
constexpr uint16_t MOTION_STOP = 0x0020;
constexpr uint16_t MOTION_ESTOP = 0x0040;

constexpr uint16_t STATUS_IN_POS = (1U << 0);
constexpr uint16_t STATUS_RUNNING = (1U << 2);
constexpr uint16_t STATUS_ALARM = (1U << 3);
constexpr uint16_t DI_X1 = (1U << 1); // P-OT side, method 18
constexpr uint16_t DI_X2 = (1U << 2); // N-OT side, method 17
constexpr uint16_t DI_LIMIT_MASK = (DI_X1 | DI_X2);

constexpr uint16_t SEEK_ACCEL_MS = 800;
constexpr uint16_t MID_ACCEL_MS = 400;
constexpr uint16_t FOLLOW_ACCEL_MS = 400;
constexpr uint32_t REPLY_TIMEOUT_MS = 300;
constexpr uint32_t SEND_GAP_MS = 50;
constexpr uint32_t STALL_MS = 5000;
constexpr uint32_t SEEK_GRACE_MS = 2000;
constexpr uint32_t HIT_SETTLE_MS = 600;
constexpr uint8_t LIMIT_HIT_SAMPLES = 2;
constexpr uint8_t RECOVERY_PER_LEG = 5;
// Recovery may need to cross nearly full lock-to-lock; the far switch is often well past 25k.
constexpr int32_t RECOVERY_MAX_TRAVEL = 150000;
// Ignore the opposite DI until we have actually left the near-end zone.
constexpr int32_t RECOVERY_OTHER_MIN = 8000;
// OT before this travel from seek origin is treated as a false/early trip: clear and crawl on.
constexpr int32_t EARLY_OT_TRAVEL = 30000;
constexpr uint32_t BUTTON_LOCKOUT_MS = 300;
constexpr uint32_t HOME_TIMEOUT_DEFAULT_MS = 90000;
constexpr uint32_t LINK_OK_DEFAULT_MS = 2000;
constexpr uint8_t TX_TRIES = 8;
constexpr int32_t MOTION_PULSES = 20;
constexpr int32_t SEEK_CHUNK = 100000;
constexpr int32_t MIN_TRAVEL = 1000;
constexpr int32_t BACKOFF_RELEASE = 1500;
constexpr int32_t BACKOFF_MISS = 8000;
constexpr int32_t BACKOFF_MAX = 12000;
constexpr int32_t MID_TOL = 800;
constexpr uint32_t MID_STALL_MS = 5000;
constexpr uint8_t MID_REL_RETRIES = 6;

int32_t pulses_apart(int32_t a, int32_t b)
{
    const int64_t d = (int64_t)a - (int64_t)b;
    return (int32_t)(d < 0 ? -d : d);
}
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
    // @Description: Unused. Kept so existing parameter storage indices stay valid.
    // @User: Advanced
    AP_GROUPINFO("REG_ADDR", 2, AP_ModbusSteering, reg_address, 52),

    // @Param: MAX_STEPS
    // @DisplayName: Measured half travel
    // @Description: Half of the measured lock-to-lock travel, in pulses. Written when calibration reaches the centre.
    // @User: Standard
    AP_GROUPINFO("MAX_STEPS", 3, AP_ModbusSteering, max_steps, 200000),

    // @Param: START_SPD
    // @DisplayName: Move-to-centre start speed
    // @Description: CL57R register 0x0030 while moving from the second limit to mid-travel.
    // @Units: RPM
    // @Range: 2 300
    // @User: Standard
    AP_GROUPINFO("START_SPD", 4, AP_ModbusSteering, start_speed, 15),

    // @Param: MAX_SPD
    // @DisplayName: Stick follow speed
    // @Description: CL57R register 0x0033 while following the stick after calibration.
    // @Units: RPM
    // @Range: 1 3000
    // @User: Standard
    AP_GROUPINFO("MAX_SPD", 5, AP_ModbusSteering, max_speed, 1300),

    // @Param: POS_DB
    // @DisplayName: Stick retarget deadband
    // @Description: Skip a new position command if the stick target moved less than this many pulses.
    // @User: Standard
    AP_GROUPINFO("POS_DB", 6, AP_ModbusSteering, pos_db, 500),

    // @Param: RET_SLEW
    // @DisplayName: Return slew
    // @Description: Unused. Kept so existing parameter storage indices stay valid.
    // @User: Advanced
    AP_GROUPINFO("RET_SLEW", 7, AP_ModbusSteering, ret_slew, 0),

    // @Param: OUT_REV
    // @DisplayName: Expected lock-to-lock turns
    // @Description: Output-shaft turns lock to lock. With RATIO, rejects a measured travel that is far from OUT_REV*RATIO*4000 pulses. 0 disables that check.
    // @Units: rev
    // @Range: 0 20
    // @User: Standard
    AP_GROUPINFO("OUT_REV", 8, AP_ModbusSteering, out_rev, 4),

    // @Param: RATIO
    // @DisplayName: Gearbox Ratio
    // @Description: Motor revolutions per output revolution. Used only to check the measured travel.
    // @Range: 1 200
    // @User: Standard
    AP_GROUPINFO("RATIO", 9, AP_ModbusSteering, ratio, 25),

    // @Param: RST_CH
    // @DisplayName: Alarm reset RC channel
    // @Description: RC channel that clears the CL57R alarm on a rising edge (PWM above 1800). 0 disables the button.
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("RST_CH", 10, AP_ModbusSteering, rst_ch, 0),

    // @Param: CAL_CH
    // @DisplayName: Calibration RC channel
    // @Description: RC channel that starts limit calibration on a rising edge (PWM above 1800). Ignored while armed. 0 disables.
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("CAL_CH", 11, AP_ModbusSteering, cal_ch, 0),

    // @Param: CAL_MTH
    // @DisplayName: First limit method
    // @Description: CL57R home method for the first end (register 0x0040). 17 seeks the negative limit X2. 18 seeks the positive limit X1. The other end is run second.
    // @Values: 17:NegativeLimit,18:PositiveLimit
    // @Range: 17 18
    // @User: Standard
    AP_GROUPINFO("CAL_MTH", 12, AP_ModbusSteering, cal_mth, 17),

    // @Param: HOME_SPD
    // @DisplayName: Home seek speed
    // @Description: Fast approach speed written to CL57R 0x0041. Also the speed of the move from the second limit to mid-travel.
    // @Units: RPM
    // @Range: 5 3000
    // @User: Standard
    AP_GROUPINFO("HOME_SPD", 13, AP_ModbusSteering, home_speed, 300),

    // @Param: CAL_TRIG
    // @DisplayName: Calibration trigger
    // @Description: 1 starts limit calibration. 2 clears the drive alarm and stops motion. Ignored while armed. Resets to 0 when accepted.
    // @Values: 0:None,1:Calibrate,2:ClearAlarm
    // @User: Standard
    AP_GROUPINFO("CAL_TRIG", 14, AP_ModbusSteering, cal_trig, 0),

    // @Param: CAL_MODE
    // @DisplayName: Calibration mode
    // @Description: Unused. Calibration always uses both limit switches. Kept so existing parameter storage indices stay valid.
    // @User: Advanced
    AP_GROUPINFO("CAL_MODE", 15, AP_ModbusSteering, cal_mode, 1),

    // @Param: CRAWL_SPD
    // @DisplayName: Home crawl and backoff speed
    // @Description: Written to CL57R 0x0042 (5..300). Also the speed used to leave a mechanical stop, and the seek speed of the second attempt.
    // @Units: RPM
    // @Range: 5 300
    // @User: Standard
    AP_GROUPINFO("CRAWL_SPD", 16, AP_ModbusSteering, crawl_speed, 60),

    // @Param: LINK_TO
    // @DisplayName: Modbus link timeout
    // @Description: Arming requires a valid Modbus reply within this many milliseconds. 0 uses 2000.
    // @Units: ms
    // @Range: 0 30000
    // @User: Standard
    AP_GROUPINFO("LINK_TO", 17, AP_ModbusSteering, link_timeout, 1500),

    // @Param: CAL_TO
    // @DisplayName: Calibration step timeout
    // @Description: Abort a seek, backoff, or move-to-centre that runs longer than this many seconds. 0 keeps 90 seconds.
    // @Units: s
    // @Range: 0 600
    // @User: Advanced
    AP_GROUPINFO("CAL_TO", 18, AP_ModbusSteering, cal_timeout_s, 0),

    // @Param: DI_INV
    // @DisplayName: Invert limit switches
    // @Description: 0: X1/X2 active when closed (NO). 1: active when open (NC). Polarity is written only to CL57R 0x0010; 0x0005 is then read as-is (this drive applies 0x0010 to the DI word, so a second software XOR would invert twice).
    // @Values: 0:Normal,1:InvertX1X2
    // @User: Standard
    AP_GROUPINFO("DI_INV", 19, AP_ModbusSteering, di_inv, 0),

    AP_GROUPEND
};

AP_ModbusSteering::AP_ModbusSteering()
{
    AP_Param::setup_object_defaults(this, var_info);
    _phase = Phase::INIT_ENABLE;
    _method = 17;
}

bool AP_ModbusSteering::homing() const
{
    return !in_init() && _phase != Phase::IDLE;
}

uint32_t AP_ModbusSteering::link_timeout_ms() const
{
    const int16_t to = link_timeout.get();
    if (to <= 0) {
        return LINK_OK_DEFAULT_MS;
    }
    return (uint32_t)to;
}

uint32_t AP_ModbusSteering::home_timeout_ms() const
{
    const int16_t sec = cal_timeout_s.get();
    if (sec <= 0) {
        return HOME_TIMEOUT_DEFAULT_MS;
    }
    return (uint32_t)sec * 1000U;
}

bool AP_ModbusSteering::link_ok() const
{
    if (_uart == nullptr || !_ever_rx) {
        return false;
    }
    return (AP_HAL::millis() - _last_rx_ms) <= link_timeout_ms();
}

uint16_t AP_ModbusSteering::home_rpm() const
{
    int16_t rpm = home_speed.get();
    if (rpm < 5) {
        rpm = 120;
    }
    if (rpm > 3000) {
        rpm = 3000;
    }
    return (uint16_t)rpm;
}

uint16_t AP_ModbusSteering::crawl_rpm() const
{
    int16_t rpm = crawl_speed.get();
    if (rpm < 5) {
        rpm = 60;
    }
    if (rpm > 300) {
        rpm = 300;
    }
    return (uint16_t)rpm;
}

uint16_t AP_ModbusSteering::seek_rpm() const
{
    if (_in_recovery || _leg_retries > 0) {
        return crawl_rpm();
    }
    return home_rpm();
}

uint16_t AP_ModbusSteering::follow_rpm() const
{
    int16_t rpm = max_speed.get();
    if (rpm < 1) {
        rpm = 120;
    }
    if (rpm > 3000) {
        rpm = 3000;
    }
    return (uint16_t)rpm;
}

int16_t AP_ModbusSteering::backoff_rpm() const
{
    const int16_t mag = (int16_t)crawl_rpm();
    // Inverted axis: method 17 seeks positive (X2). Leave that stop toward negative.
    return _method == 17 ? (int16_t)-mag : mag;
}

int32_t AP_ModbusSteering::expected_full_travel() const
{
    if (out_rev.get() <= 0 || ratio.get() <= 0) {
        return 0;
    }
    return (int32_t)out_rev.get() * (int32_t)ratio.get() * STEPS_PER_REV;
}

int32_t AP_ModbusSteering::stick_target_pulses() const
{
    int32_t half = max_steps.get();
    if (half < 1) {
        half = 1;
    }
    const float stick = constrain_float(_stick, -1.0f, 1.0f);
    return constrain_int32((int32_t)(stick * (float)half), -half, half);
}

uint16_t AP_ModbusSteering::polarity_word() const
{
    // 0x0010 bit1 = X1, bit2 = X2. 1 = inverted active level. RAM only.
    return di_inv.get() != 0 ? DI_LIMIT_MASK : 0;
}

uint16_t AP_ModbusSteering::own_limit_bit() const
{
    return (_method == 18) ? DI_X1 : DI_X2;
}

bool AP_ModbusSteering::limit_active() const
{
    return (_di_word & own_limit_bit()) != 0;
}

bool AP_ModbusSteering::x1_active() const
{
    return (_di_word & DI_X1) != 0;
}

bool AP_ModbusSteering::x2_active() const
{
    return (_di_word & DI_X2) != 0;
}

bool AP_ModbusSteering::both_limits() const
{
    return (_di_word & DI_LIMIT_MASK) == DI_LIMIT_MASK;
}

void AP_ModbusSteering::enter(Phase phase)
{
    _phase = phase;
    _expect = RxExpect::NONE;
    _waiting = false;
    _tx_tries = 0;
    _got_echo = false;
    _got_status = false;
    _got_di = false;
    _got_encoder = false;
    _phase_ms = AP_HAL::millis();
    if (phase == Phase::SEEK_POLL || phase == Phase::BACK_POLL || phase == Phase::MID_POLL) {
        _poll_step = 0;
        _last_motion_ms = _phase_ms;
    }
    // Leg metrics reset once per approach, not on every relative chunk.
    // Early-OT crawl-on keeps the same origin so "traveled" still grows toward the far switch.
    if (phase == Phase::SEEK_STOP) {
        if (!_keep_seek_origin) {
            _seek_origin = _actual_pulses;
            _leg_peak = 0;
            _saw_motion = false;
            _leg_start_ms = _phase_ms;
        }
        _keep_seek_origin = false;
        _saw_limit = false;
        _di_ready = false;
        _limit_at_start = false;
        _limit_hit_count = 0;
        _limit_was_active = false;
        _fault_code = 0;
    }
    if (phase == Phase::BACK_POLL) {
        _back_origin = _actual_pulses;
    }
    if (phase == Phase::SEEK_HIT_WAIT) {
        _poll_step = 0;
    }
}

void AP_ModbusSteering::init(AP_SerialManager &serial_manager)
{
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ModbusSteer, 0);
    if (_uart != nullptr) {
        _uart->begin(115200);
        const int8_t port = serial_manager.find_portnum(AP_SerialManager::SerialProtocol_ModbusSteer, 0);
        if (port >= 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: using SERIAL%d", (int)port);
        }
    }
}

void AP_ModbusSteering::send_u16(uint16_t reg, uint16_t value)
{
    uint8_t tx_packet[8];
    modbus_create_write_packet((uint8_t)slave_id.get(), reg, value, tx_packet);
    _uart->write(tx_packet, 8);
    arm_wait(RxExpect::WRITE);
}

void AP_ModbusSteering::send_s16(uint16_t reg, int16_t value)
{
    send_u16(reg, (uint16_t)value);
}

void AP_ModbusSteering::send_read(uint16_t reg, uint16_t count, RxExpect expect)
{
    uint8_t tx_packet[8];
    modbus_create_read_packet((uint8_t)slave_id.get(), reg, count, tx_packet);
    _uart->write(tx_packet, 8);
    arm_wait(expect);
}

void AP_ModbusSteering::send_target(int32_t target)
{
    uint8_t tx_packet[16];
    uint16_t values[2];
    values[0] = (uint16_t)(((uint32_t)target >> 16) & 0xFFFFU);
    values[1] = (uint16_t)((uint32_t)target & 0xFFFFU);
    modbus_create_write_multiple_packet((uint8_t)slave_id.get(), REG_TARGET_POS, 2, values, tx_packet);
    _uart->write(tx_packet, 13);
    arm_wait(RxExpect::WRITE);
}

void AP_ModbusSteering::arm_wait(RxExpect expect)
{
    _expect = expect;
    _waiting = true;
    _wait_ms = AP_HAL::millis();
    _last_send_ms = _wait_ms;
    _got_echo = false;
    _got_status = false;
    _got_di = false;
    _got_encoder = false;
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
    if ((fn & 0x80U) != 0) {
        return 5;
    }
    return 0;
}

void AP_ModbusSteering::note_di(uint16_t raw)
{
    // 0x0010 polarity is applied by the drive to 0x0005 on this CL57R, so
    // treat the register as the effective limit word after SEEK/INIT_POL.
    const uint16_t di = raw;
    if (!_di_log_inited || di != _di_log_eff || raw != _di_log_raw) {
        _di_log_inited = true;
        _di_log_eff = di;
        _di_log_raw = raw;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: DI=%04x%s",
                      (unsigned)di, di_inv.get() != 0 ? " (NC pol)" : "");
    }
    _di_raw = raw;
    _di_word = di;
    _got_di = true;
}

void AP_ModbusSteering::note_encoder(int32_t enc, uint32_t now)
{
    if (pulses_apart(enc, _actual_pulses) > MOTION_PULSES) {
        _saw_motion = true;
        _last_motion_ms = now;
    }
    _actual_pulses = enc;
    _got_encoder = true;
    if (_phase == Phase::SEEK_POLL) {
        const int32_t disp = pulses_apart(enc, _seek_origin);
        if (disp > _leg_peak) {
            _leg_peak = disp;
        }
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
        const uint16_t received_crc = (uint16_t)((_rx_buf[offset + frame_len - 1] << 8) |
                                                  _rx_buf[offset + frame_len - 2]);
        if (modbus_crc16(&_rx_buf[offset], frame_len - 2) != received_crc) {
            offset++;
            continue;
        }

        _ever_rx = true;
        _last_rx_ms = AP_HAL::millis();
        const uint8_t fn = _rx_buf[offset + 1];
        if (fn == 0x06 || fn == 0x10 || (fn & 0x80U) != 0) {
            _got_echo = true;
        } else if (fn == 0x03 && frame_len >= 7) {
            const uint8_t nbytes = _rx_buf[offset + 2];
            if (_expect == RxExpect::STATUS && nbytes >= 2) {
                _status_word = (uint16_t)((_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4]);
                _got_status = true;
                if (nbytes >= 4 && frame_len >= 9) {
                    _fault_code = (uint16_t)((_rx_buf[offset + 5] << 8) | _rx_buf[offset + 6]);
                }
            } else if (_expect == RxExpect::DI && nbytes >= 2) {
                const uint16_t raw = (uint16_t)((_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4]);
                note_di(raw);
            } else if (_expect == RxExpect::ENCODER && nbytes >= 4 && frame_len >= 9) {
                const uint16_t high = (uint16_t)((_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4]);
                const uint16_t low = (uint16_t)((_rx_buf[offset + 5] << 8) | _rx_buf[offset + 6]);
                note_encoder((int32_t)(((uint32_t)high << 16) | low), _last_rx_ms);
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

bool AP_ModbusSteering::take_reply()
{
    switch (_expect) {
    case RxExpect::WRITE:
        return _got_echo;
    case RxExpect::STATUS:
        return _got_status;
    case RxExpect::DI:
        return _got_di;
    case RxExpect::ENCODER:
        return _got_encoder;
    case RxExpect::NONE:
        break;
    }
    return false;
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

void AP_ModbusSteering::start_home()
{
    if (hal.util->get_soft_armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: home ignored, armed");
        return;
    }
    if (seeking()) {
        return;
    }
    _homed = false;
    _follow_profile_ok = false;
    _follow_force = false;
    _follow_via_stop = false;
    _follow_limit_hold = false;
    _await_stick_center = false;
    _keep_seek_origin = false;
    _leg = 0;
    _leg_retries = 0;
    _limit_hit_count = 0;
    _in_recovery = false;
    _end_from_recovery = false;
    _recovery_saw_active = false;
    _recovery_used = 0;
    _recovery_bit = 0;
    _end_pos[0] = 0;
    _end_pos[1] = 0;
    _method = cal_mth.get() == 18 ? 18 : 17;
    if (following()) {
        _home_pending = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home start M%u", (unsigned)_method);
        enter(Phase::SEEK_STOP);
        return;
    }
    if (in_init() || _phase != Phase::IDLE) {
        _home_pending = true;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home queued");
        return;
    }
    _home_pending = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home start M%u", (unsigned)_method);
    enter(Phase::SEEK_STOP);
}

void AP_ModbusSteering::abort_home(const char *reason, bool estop)
{
    _homed = false;
    _home_pending = false;
    _in_recovery = false;
    _end_from_recovery = false;
    _limit_hit_count = 0;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: %s", reason);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: di=%04x st=%04x fault=%u",
                  (unsigned)_di_word, (unsigned)_status_word, (unsigned)_fault_code);
    if (cal_trig.get() != 0) {
        cal_trig.set_and_save(0);
        _cal_trig_last = 0;
    }
    enter(Phase::ABORT_STOP);
    _use_estop = estop;
}

void AP_ModbusSteering::begin_backoff(const char *why)
{
    // Used only to leave a switch we are already sitting on at seek start.
    _back_from_limit = limit_active();
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: %s, back off M%u", why, (unsigned)_method);
    enter(Phase::BACK_STOP);
}

void AP_ModbusSteering::begin_hard_stop_recovery()
{
    if (_recovery_used >= RECOVERY_PER_LEG) {
        abort_home("recovery exhausted", true);
        return;
    }
    if (_in_recovery) {
        // Already recovering and hit the stop again — do not keep ramming.
        abort_home("recovery still on hard stop", true);
        return;
    }
    _recovery_used++;
    _hard_stop_pos = _actual_pulses;
    _recovery_origin = _actual_pulses;
    _failed_method = _method;
    _recovery_bit = own_limit_bit();
    _recovery_saw_active = (_di_word & _recovery_bit) != 0;
    _in_recovery = true;
    _end_from_recovery = false;
    _leg_retries = 1; // crawl while recovering
    _method = (_method == 17) ? 18 : 17;
    _use_estop = true; // hard stop first, then clear, then leave opposite
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: recovery hard stop → M%u",
                  (unsigned)_method);
    enter(Phase::BACK_STOP);
}

void AP_ModbusSteering::mark_end(bool falling_proxy)
{
    if (_leg > 1) {
        return;
    }
    _end_pos[_leg] = _actual_pulses;
    _saw_limit = true;
    _end_from_recovery = falling_proxy;
    if (falling_proxy) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: end%u falling proxy @ %d",
                      (unsigned)_leg, (int)_actual_pulses);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: end%u rising M%u @ %d",
                      (unsigned)_leg, (unsigned)_method, (int)_actual_pulses);
    }
    enter(Phase::SEEK_HIT_STOP);
}

void AP_ModbusSteering::complete_end()
{
    if (_leg == 0) {
        _leg = 1;
        _leg_retries = 0;
        _recovery_used = 0; // fresh budget for the second end
        _in_recovery = false;
        // Recovery already reversed method to leave the missed switch.
        if (!_end_from_recovery) {
            _method = (_method == 17) ? 18 : 17;
        }
        _end_from_recovery = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: seek M%u", (unsigned)_method);
        enter(Phase::SEEK_STOP);
        return;
    }

    const int32_t travel = pulses_apart(_end_pos[0], _end_pos[1]);
    const int32_t expected = expected_full_travel();
    if (travel < MIN_TRAVEL) {
        abort_home("travel too small");
        return;
    }
    if (expected >= 20000 && (travel > expected * 5 || travel < expected / 4)) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: travel %d expected %d",
                      (int)travel, (int)expected);
        abort_home("travel implausible");
        return;
    }
    const int32_t half = travel / 2;
    max_steps.set_and_save(half);
    _travel_pulses = travel;
    // Absolute mid; motion uses a relative crawl step from the live encoder.
    _mid_target = (int32_t)(((int64_t)_end_pos[0] + (int64_t)_end_pos[1]) / 2);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: travel %d mid %d",
                  (int)travel, (int)_mid_target);
    _in_recovery = false;
    _end_from_recovery = false;
    _mid_alarm_retried = false;
    _mid_rel_retries = 0;
    enter(Phase::MID_STOP);
}

void AP_ModbusSteering::finish_mid_zero()
{
    const int32_t err = pulses_apart(_actual_pulses, _mid_target);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: mid accept @ %d (target %d, err %d)",
                  (int)_actual_pulses, (int)_mid_target, (int)err);
    enter(Phase::MID_ZERO);
}

void AP_ModbusSteering::finish_to_idle()
{
    if (_home_pending && !hal.util->get_soft_armed()) {
        _home_pending = false;
        _homed = false;
        _leg = 0;
        _leg_retries = 0;
        _method = cal_mth.get() == 18 ? 18 : 17;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home start M%u", (unsigned)_method);
        enter(Phase::SEEK_STOP);
        return;
    }
    _home_pending = false;
    enter(Phase::IDLE);
}

void AP_ModbusSteering::poll_triggers()
{
    const int8_t trig = cal_trig.get();
    if (!_cal_trig_inited) {
        _cal_trig_last = trig;
        _cal_trig_inited = true;
    } else if (trig != 0 && trig != _cal_trig_last) {
        _cal_trig_last = trig;
        if (hal.util->get_soft_armed()) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: CAL_TRIG ignored, armed");
            cal_trig.set_and_save(0);
            _cal_trig_last = 0;
        } else if (trig == 1) {
            start_home();
            // Always clear so a later QGC write of 1 is a fresh edge.
            if (cal_trig.get() != 0) {
                cal_trig.set_and_save(0);
                _cal_trig_last = 0;
            }
        } else if (trig == 2) {
            _home_pending = false;
            if (seeking()) {
                _homed = false;
            }
            if (cal_trig.get() != 0) {
                cal_trig.set_and_save(0);
                _cal_trig_last = 0;
            }
            if (!in_init()) {
                enter(Phase::ABORT_STOP);
            }
        } else {
            cal_trig.set_and_save(0);
            _cal_trig_last = 0;
        }
    } else if (trig == 0) {
        _cal_trig_last = 0;
    }

    const uint32_t now = AP_HAL::millis();
    const bool rst_edge = rc_rising_edge(rst_ch.get(), _rst_was_high);
    const bool home_edge = rc_rising_edge(cal_ch.get(), _cal_was_high);
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
        _home_pending = false;
        if (seeking()) {
            _homed = false;
        }
        if (!in_init()) {
            enter(Phase::ABORT_STOP);
        }
    }
}

void AP_ModbusSteering::advance_setup()
{
    switch (_phase) {
    case Phase::INIT_ENABLE:
        enter(Phase::INIT_CLEAR);
        break;
    case Phase::INIT_CLEAR:
        enter(Phase::INIT_X1);
        break;
    case Phase::INIT_X1:
        enter(Phase::INIT_X2);
        break;
    case Phase::INIT_X2:
        enter(Phase::INIT_POL);
        break;
    case Phase::INIT_POL:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: ready%s", di_inv.get() != 0 ? " DI inv" : "");
        finish_to_idle();
        break;
    case Phase::SEEK_STOP:
        enter(Phase::SEEK_CLEAR);
        break;
    case Phase::SEEK_CLEAR:
        enter(Phase::SEEK_DISABLE);
        break;
    case Phase::SEEK_DISABLE:
        enter(Phase::SEEK_ENABLE);
        break;
    case Phase::SEEK_ENABLE:
        enter(Phase::SEEK_X1);
        break;
    case Phase::SEEK_X1:
        enter(Phase::SEEK_X2);
        break;
    case Phase::SEEK_X2:
        enter(Phase::SEEK_POL);
        break;
    case Phase::SEEK_POL:
        enter(Phase::SEEK_STARTSPD);
        break;
    case Phase::SEEK_STARTSPD:
        enter(Phase::SEEK_ACCEL);
        break;
    case Phase::SEEK_ACCEL:
        enter(Phase::SEEK_DECEL);
        break;
    case Phase::SEEK_DECEL:
        enter(Phase::SEEK_MAXSPD);
        break;
    case Phase::SEEK_MAXSPD:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: seek %u M%u %urpm (DI)",
                      (unsigned)(_leg + 1), (unsigned)_method, (unsigned)seek_rpm());
        enter(Phase::SEEK_TARGET);
        break;
    case Phase::SEEK_TARGET:
        enter(Phase::SEEK_RUN);
        break;
    case Phase::SEEK_RUN:
        enter(Phase::SEEK_POLL);
        break;
    case Phase::SEEK_HIT_STOP:
        enter(Phase::SEEK_HIT_WAIT);
        break;
    case Phase::SEEK_HIT_WAIT:
        break;
    case Phase::BACK_STOP:
        enter(Phase::BACK_CLEAR);
        break;
    case Phase::BACK_CLEAR:
        enter(Phase::BACK_ENABLE);
        break;
    case Phase::BACK_ENABLE:
        if (_in_recovery) {
            // Estop+clear only — leave the stop via SEEK opposite, not speed-mode rams.
            enter(Phase::BACK_HALT);
        } else {
            enter(Phase::BACK_SPD);
        }
        break;
    case Phase::BACK_SPD:
        enter(Phase::BACK_START);
        break;
    case Phase::BACK_START:
        enter(Phase::BACK_POLL);
        break;
    case Phase::BACK_HALT:
        // After start-on-limit leave, or after recovery clear, resume seek.
        enter(Phase::SEEK_STOP);
        break;
    case Phase::MID_STOP:
        enter(Phase::MID_CLEAR);
        break;
    case Phase::MID_CLEAR:
        enter(Phase::MID_ENABLE);
        break;
    case Phase::MID_ENABLE:
        enter(Phase::MID_STARTSPD);
        break;
    case Phase::MID_STARTSPD:
        enter(Phase::MID_ACCEL);
        break;
    case Phase::MID_ACCEL:
        enter(Phase::MID_DECEL);
        break;
    case Phase::MID_DECEL:
        enter(Phase::MID_MAXSPD);
        break;
    case Phase::MID_MAXSPD:
        enter(Phase::MID_TARGET);
        break;
    case Phase::MID_TARGET:
        enter(Phase::MID_RUN);
        break;
    case Phase::MID_RUN:
        enter(Phase::MID_POLL);
        break;
    case Phase::MID_ZERO:
        _homed = true;
        _follow_cmd_target = 0;
        _follow_target = 0;
        _follow_profile_ok = false;
        _follow_force = false;
        _follow_via_stop = false;
        _follow_limit_hold = false;
        // Hold mid until stick is near center so RC failsafe extremes do not slam a rail.
        _await_stick_center = true;
        if (cal_trig.get() != 0) {
            cal_trig.set_and_save(0);
            _cal_trig_last = 0;
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: calibrated at mid");
        enter(Phase::ALARM_CLEAR);
        break;
    case Phase::FOLLOW_STOP:
        if (_follow_limit_hold) {
            _follow_limit_hold = false;
            enter(Phase::IDLE);
            break;
        }
        if (_follow_profile_ok) {
            enter(Phase::FOLLOW_TARGET);
        } else {
            enter(Phase::FOLLOW_STARTSPD);
        }
        break;
    case Phase::FOLLOW_STARTSPD:
        enter(Phase::FOLLOW_ACCEL);
        break;
    case Phase::FOLLOW_ACCEL:
        enter(Phase::FOLLOW_DECEL);
        break;
    case Phase::FOLLOW_DECEL:
        enter(Phase::FOLLOW_MAXSPD);
        break;
    case Phase::FOLLOW_MAXSPD:
        _follow_profile_ok = true;
        enter(Phase::FOLLOW_TARGET);
        break;
    case Phase::FOLLOW_TARGET:
        enter(Phase::FOLLOW_RUN);
        break;
    case Phase::FOLLOW_RUN:
        _follow_cmd_target = _follow_target;
        _follow_force = false;
        _follow_via_stop = false;
        enter(Phase::IDLE);
        break;
    case Phase::ABORT_STOP:
        enter(Phase::ALARM_CLEAR);
        break;
    case Phase::ALARM_CLEAR:
        enter(Phase::ALARM_ENABLE);
        break;
    case Phase::ALARM_ENABLE:
        finish_to_idle();
        break;
    case Phase::IDLE:
    case Phase::SEEK_POLL:
    case Phase::BACK_POLL:
    case Phase::MID_POLL:
        break;
    }
}

bool AP_ModbusSteering::try_begin_follow()
{
    if (!_homed || alarmed() || !link_ok()) {
        return false;
    }

    // After calibration stay at mid until the stick returns near zero.
    if (_await_stick_center) {
        if (fabsf(_stick) > 0.15f) {
            return false;
        }
        _await_stick_center = false;
        _follow_cmd_target = 0;
        _follow_target = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: follow armed");
        return false;
    }

    int32_t desired = stick_target_pulses();
    // Hard stop on DI: never command further into an active limit.
    // Inverted axis: + toward X2, − toward X1.
    if (x2_active() && desired > _actual_pulses) {
        desired = _actual_pulses;
    }
    if (x1_active() && desired < _actual_pulses) {
        desired = _actual_pulses;
    }
    if (x2_active() && _stick > 0.05f && desired >= _actual_pulses) {
        return false;
    }
    if (x1_active() && _stick < -0.05f && desired <= _actual_pulses) {
        return false;
    }

    int32_t db = pos_db.get();
    if (db < 0) {
        db = 0;
    }
    if (!_follow_force && pulses_apart(desired, _follow_cmd_target) <= db) {
        return false;
    }

    const bool running = (_status_word & STATUS_RUNNING) != 0;
    const bool in_pos = (_status_word & STATUS_IN_POS) != 0;
    const int32_t err_new = desired - _actual_pulses;
    const int32_t err_old = _follow_cmd_target - _actual_pulses;
    const bool reverse = running && !in_pos &&
                         ((err_new > 0) != (err_old > 0)) &&
                         (pulses_apart(desired, _actual_pulses) > db);

    // Same-direction retarget waits for rest so relative-style stacking
    // cannot accumulate. Reverse goes through STOP first.
    if (running && !in_pos && !reverse && !_follow_force) {
        return false;
    }

    _follow_target = desired;
    _follow_via_stop = reverse;
    if (reverse) {
        enter(Phase::FOLLOW_STOP);
    } else if (!_follow_profile_ok) {
        enter(Phase::FOLLOW_STARTSPD);
    } else {
        enter(Phase::FOLLOW_TARGET);
    }
    return true;
}

void AP_ModbusSteering::soft_follow_ot()
{
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: follow OT, hold @ %d",
                  (int)_actual_pulses);
    _follow_target = _actual_pulses;
    _follow_cmd_target = _actual_pulses;
    _follow_profile_ok = false;
    _follow_force = false;
    _follow_limit_hold = false;
    _use_estop = true;
    enter(Phase::ABORT_STOP);
}

void AP_ModbusSteering::evaluate_follow_guard()
{
    if (!_homed || seeking()) {
        return;
    }

    if ((_status_word & STATUS_ALARM) != 0 && (_fault_code == 1 || _fault_code == 4)) {
        soft_follow_ot();
        return;
    }

    if (!_got_di) {
        return;
    }

    const bool hit_x1 = x1_active();
    const bool hit_x2 = x2_active();
    if (!hit_x1 && !hit_x2) {
        return;
    }

    bool clamped = false;
    if (hit_x2 && _follow_target > _actual_pulses) {
        _follow_target = _actual_pulses;
        _follow_cmd_target = _actual_pulses;
        clamped = true;
    }
    if (hit_x1 && _follow_target < _actual_pulses) {
        _follow_target = _actual_pulses;
        _follow_cmd_target = _actual_pulses;
        clamped = true;
    }

    if (clamped || ((hit_x2 && _stick > 0.05f) || (hit_x1 && _stick < -0.05f))) {
        if (following() || ((_status_word & STATUS_RUNNING) != 0)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: limit hold DI=%04x",
                          (unsigned)_di_word);
            _follow_cmd_target = _actual_pulses;
            _follow_target = _actual_pulses;
            _follow_via_stop = false;
            _follow_limit_hold = true;
            enter(Phase::FOLLOW_STOP);
        }
    }
}

void AP_ModbusSteering::evaluate_seek(uint32_t now)
{
    if (_got_di && both_limits()) {
        abort_home("both limits, check DI_INV");
        return;
    }

    if (_in_recovery) {
        // Leave the hard stop; latch end only on falling of the missed switch.
        // Never invent an end from the hard-stop position alone.
        const int32_t left = pulses_apart(_actual_pulses, _recovery_origin);
        int32_t window = RECOVERY_MAX_TRAVEL;
        const int32_t exp = expected_full_travel();
        if (exp > window) {
            window = exp + 20000;
        }
        const bool past_window = left >= window;
        // Opposite DI near the origin is usually the first switch we just left — keep going.
        const bool hit_other = left >= RECOVERY_OTHER_MIN &&
                               _got_di &&
                               ((_di_word & DI_LIMIT_MASK & ~_recovery_bit) != 0);
        if (past_window || hit_other) {
            _in_recovery = false;
            _end_from_recovery = false;
            _method = _failed_method;
            _leg_retries = 1;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: recovery reseek M%u for DI",
                          (unsigned)_method);
            enter(Phase::SEEK_STOP);
            return;
        }
        if (_got_di) {
            const bool active = (_di_word & _recovery_bit) != 0;
            if (active) {
                _recovery_saw_active = true;
            } else if (_recovery_saw_active) {
                _in_recovery = false;
                mark_end(true);
                return;
            }
        }
    } else if (_got_di) {
        if (!_di_ready) {
            _di_ready = true;
            _limit_at_start = limit_active();
            if (_limit_at_start) {
                // Already on own switch — that is this end. Do not shove off it.
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: already on limit M%u",
                              (unsigned)_method);
                mark_end(false);
                return;
            }
        }
        if (limit_active()) {
            _limit_hit_count++;
            if (_limit_hit_count >= LIMIT_HIT_SAMPLES) {
                mark_end(false);
                return;
            }
        } else {
            _limit_hit_count = 0;
        }
    }

    if ((now - _leg_start_ms) > home_timeout_ms()) {
        abort_home("seek timeout");
        return;
    }

    const bool alarm = (_status_word & STATUS_ALARM) != 0;
    if (alarm && (_fault_code == 1 || _fault_code == 4)) {
        if (_in_recovery) {
            // Opposite move also hit the stop — stop hammering.
            abort_home("recovery still on hard stop", true);
            return;
        }
        // Far switch can sit well past the near end. Early OT is usually a false trip —
        // clear and keep crawling the same way instead of reversing into the first switch.
        const int32_t traveled = pulses_apart(_actual_pulses, _seek_origin);
        int32_t need = EARLY_OT_TRAVEL;
        const int32_t exp = expected_full_travel();
        if (exp > 0 && (exp / 3) > need) {
            need = exp / 3;
        }
        if (traveled < need) {
            _leg_retries = 1;
            _keep_seek_origin = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "CL57R: early OT @%d (need %d), crawl on M%u",
                          (int)_actual_pulses, (int)need, (unsigned)_method);
            _use_estop = true;
            enter(Phase::SEEK_STOP);
            return;
        }
        begin_hard_stop_recovery();
        return;
    }

    // Chunk finished without hitting a switch: issue another relative move.
    if (_got_status && (_status_word & STATUS_IN_POS) != 0 && (now - _phase_ms) > 200U) {
        enter(Phase::SEEK_TARGET);
        return;
    }
    if (!_in_recovery &&
        (now - _phase_ms) > SEEK_GRACE_MS && (now - _last_motion_ms) > STALL_MS) {
        if (limit_active()) {
            mark_end(false);
        } else {
            const int32_t traveled = pulses_apart(_actual_pulses, _seek_origin);
            int32_t need = EARLY_OT_TRAVEL;
            const int32_t exp = expected_full_travel();
            if (exp > 0 && (exp / 3) > need) {
                need = exp / 3;
            }
            if (traveled < need) {
                _leg_retries = 1;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: early stall @%d, crawl on M%u",
                              (int)_actual_pulses, (unsigned)_method);
                enter(Phase::SEEK_TARGET);
            } else {
                begin_hard_stop_recovery();
            }
        }
    }
}

void AP_ModbusSteering::evaluate_back(uint32_t now)
{
    if ((now - _phase_ms) > home_timeout_ms()) {
        abort_home("backoff timeout", true);
        return;
    }
    if ((_status_word & STATUS_ALARM) != 0 && (_fault_code == 1 || _fault_code == 4)) {
        if (_in_recovery) {
            // Do not re-command speed into the stop. Clear once, then seek opposite.
            enter(Phase::BACK_HALT);
            return;
        }
        abort_home("backoff alarm", true);
        return;
    }
    if ((now - _phase_ms) > SEEK_GRACE_MS && (now - _last_motion_ms) > STALL_MS) {
        if (_in_recovery) {
            enter(Phase::BACK_HALT);
            return;
        }
        abort_home("backoff stalled", true);
        return;
    }
    const int32_t traveled = pulses_apart(_actual_pulses, _back_origin);
    bool done = false;
    if (_in_recovery) {
        // Brief leave nudge, then SEEK_POLL watches falling DI at crawl.
        done = traveled >= BACKOFF_RELEASE || (now - _phase_ms) > 500U;
    } else if (_back_from_limit) {
        const bool released = _got_di && !limit_active();
        done = (released && traveled >= BACKOFF_RELEASE) || traveled >= BACKOFF_MAX;
    } else {
        done = traveled >= BACKOFF_MISS;
    }
    if (done) {
        enter(Phase::BACK_HALT);
    }
}

void AP_ModbusSteering::evaluate_mid(uint32_t now)
{
    const int32_t err = pulses_apart(_actual_pulses, _mid_target);

    if ((_status_word & STATUS_ALARM) != 0 && (_fault_code == 1 || _fault_code == 4)) {
        if (!_mid_alarm_retried) {
            _mid_alarm_retried = true;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: centre alarm, re-clear");
            enter(Phase::MID_STOP);
            return;
        }
        // One more REL from the live encoder after clear — never zero far from mid.
        if (_mid_rel_retries < MID_REL_RETRIES && err > MID_TOL) {
            _mid_rel_retries++;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "CL57R: centre OT, REL retry %u err=%d",
                          (unsigned)_mid_rel_retries, (int)err);
            enter(Phase::MID_STOP);
            return;
        }
        if (err <= MID_TOL) {
            finish_mid_zero();
            return;
        }
        abort_home("centre alarm");
        return;
    }

    if (err <= MID_TOL && (_status_word & STATUS_IN_POS) != 0 && (now - _phase_ms) > 300U) {
        finish_mid_zero();
        return;
    }

    // Stopped short of mid: issue another relative hop from the live encoder.
    // Soft-accepting ~5% off zeroed the frame wrong and follow then hit OT.
    if ((_status_word & STATUS_IN_POS) != 0 && err > MID_TOL && (now - _phase_ms) > 300U) {
        if (_mid_rel_retries < MID_REL_RETRIES) {
            _mid_rel_retries++;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                          "CL57R: centre correct %u err=%d",
                          (unsigned)_mid_rel_retries, (int)err);
            enter(Phase::MID_TARGET);
            return;
        }
        abort_home("centre short");
        return;
    }

    if ((now - _phase_ms) > MID_STALL_MS &&
        (now - _last_motion_ms) > STALL_MS) {
        if (err <= MID_TOL) {
            finish_mid_zero();
            return;
        }
        if (_mid_rel_retries < MID_REL_RETRIES) {
            _mid_rel_retries++;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "CL57R: centre stall, REL retry %u err=%d",
                          (unsigned)_mid_rel_retries, (int)err);
            enter(Phase::MID_STOP);
            return;
        }
        if ((now - _phase_ms) > home_timeout_ms()) {
            abort_home("centre timeout");
        }
    }
}

void AP_ModbusSteering::on_reply(uint32_t now)
{
    switch (_phase) {
    case Phase::SEEK_POLL:
        evaluate_seek(now);
        if (_phase == Phase::SEEK_POLL) {
            _poll_step++;
        }
        break;
    case Phase::SEEK_HIT_WAIT:
        // Wait for the drive to actually stop before reversing / going to mid.
        if (_got_status && (_status_word & STATUS_RUNNING) == 0 &&
            (now - _phase_ms) >= HIT_SETTLE_MS) {
            complete_end();
        } else {
            _poll_step++;
        }
        break;
    case Phase::BACK_POLL:
        evaluate_back(now);
        if (_phase == Phase::BACK_POLL) {
            _poll_step++;
        }
        break;
    case Phase::MID_POLL:
        evaluate_mid(now);
        if (_phase == Phase::MID_POLL) {
            _poll_step++;
        }
        break;
    case Phase::IDLE:
        evaluate_follow_guard();
        if (_phase == Phase::IDLE) {
            _poll_step++;
        }
        break;
    default: {
        const Phase before = _phase;
        if (following()) {
            evaluate_follow_guard();
        }
        // Guard may have switched to FOLLOW_STOP / ABORT — do not skip the stop TX.
        if (_phase == before) {
            advance_setup();
        }
        break;
    }
    }
}

void AP_ModbusSteering::send_current()
{
    int16_t start_rpm = start_speed.get();
    if (start_rpm < 2) {
        start_rpm = 15;
    }
    if (start_rpm > 300) {
        start_rpm = 300;
    }

    switch (_phase) {
    case Phase::INIT_ENABLE:
    case Phase::SEEK_ENABLE:
    case Phase::BACK_ENABLE:
    case Phase::MID_ENABLE:
    case Phase::ALARM_ENABLE:
        send_u16(REG_ENABLE, 1);
        break;
    case Phase::SEEK_DISABLE:
        send_u16(REG_ENABLE, 0);
        break;
    case Phase::INIT_CLEAR:
    case Phase::SEEK_CLEAR:
    case Phase::BACK_CLEAR:
    case Phase::MID_CLEAR:
    case Phase::ALARM_CLEAR:
        send_u16(REG_AUX, AUX_ALARM_CLEAR);
        break;
    case Phase::INIT_X1:
    case Phase::SEEK_X1:
        // Leave X1/X2 unassigned: driver OT latches and blocks travel.
        send_u16(REG_X1_FN, FN_NONE);
        break;
    case Phase::INIT_X2:
    case Phase::SEEK_X2:
        send_u16(REG_X2_FN, FN_NONE);
        break;
    case Phase::INIT_POL:
    case Phase::SEEK_POL:
        send_u16(REG_DI_POLARITY, polarity_word());
        break;
    case Phase::SEEK_STOP:
    case Phase::SEEK_HIT_STOP:
    case Phase::BACK_HALT:
    case Phase::MID_STOP:
    case Phase::FOLLOW_STOP:
        send_u16(REG_MOTION, MOTION_STOP);
        break;
    case Phase::BACK_STOP:
        // Recovery from OT must estop first so we do not keep torque into the stop.
        send_u16(REG_MOTION, (_use_estop || _in_recovery) ? MOTION_ESTOP : MOTION_STOP);
        _use_estop = false;
        break;
    case Phase::SEEK_HIT_WAIT:
        send_read(REG_STATUS, 2, RxExpect::STATUS);
        break;
    case Phase::ABORT_STOP:
        send_u16(REG_MOTION, _use_estop ? MOTION_ESTOP : MOTION_STOP);
        _use_estop = false;
        break;
    case Phase::BACK_SPD: {
        // Recovery moves the new seek direction; normal backoff leaves the current stop.
        const int16_t mag = (int16_t)crawl_rpm();
        const int16_t rpm = _in_recovery
                            ? ((_method == 17) ? mag : (int16_t)-mag)
                            : backoff_rpm();
        send_s16(REG_MAX_SPD, rpm);
        break;
    }
    case Phase::BACK_START:
        send_u16(REG_MOTION, MOTION_SPEED);
        break;
    case Phase::SEEK_STARTSPD:
    case Phase::MID_STARTSPD:
    case Phase::FOLLOW_STARTSPD:
        send_u16(REG_START_SPD, (uint16_t)start_rpm);
        break;
    case Phase::SEEK_ACCEL:
        send_u16(REG_ACCEL, SEEK_ACCEL_MS);
        break;
    case Phase::MID_ACCEL:
        send_u16(REG_ACCEL, MID_ACCEL_MS);
        break;
    case Phase::FOLLOW_ACCEL:
        send_u16(REG_ACCEL, FOLLOW_ACCEL_MS);
        break;
    case Phase::SEEK_DECEL:
        send_u16(REG_DECEL, SEEK_ACCEL_MS);
        break;
    case Phase::MID_DECEL:
        send_u16(REG_DECEL, MID_ACCEL_MS);
        break;
    case Phase::FOLLOW_DECEL:
        send_u16(REG_DECEL, FOLLOW_ACCEL_MS);
        break;
    case Phase::SEEK_MAXSPD:
        send_u16(REG_MAX_SPD, seek_rpm());
        break;
    case Phase::MID_MAXSPD:
        // HOME_SPD for mid; REL retries cover short/OT instead of crawling the whole way.
        send_u16(REG_MAX_SPD, home_rpm());
        break;
    case Phase::FOLLOW_MAXSPD:
        send_u16(REG_MAX_SPD, follow_rpm());
        break;
    case Phase::SEEK_TARGET: {
        // Inverted axis: method 18 → − toward X1; method 17 → + toward X2.
        const int32_t chunk = (_method == 18) ? -SEEK_CHUNK : SEEK_CHUNK;
        send_target(chunk);
        break;
    }
    case Phase::MID_TARGET:
        // Relative crawl from the live encoder to the remembered absolute mid.
        _mid_rel = _mid_target - _actual_pulses;
        send_target(_mid_rel);
        break;
    case Phase::FOLLOW_TARGET:
        send_target(_follow_target);
        break;
    case Phase::SEEK_RUN:
        send_u16(REG_MOTION, MOTION_REL);
        break;
    case Phase::MID_RUN:
        send_u16(REG_MOTION, MOTION_REL);
        break;
    case Phase::FOLLOW_RUN: {
        // After mid-zero the encoder frame is absolute. Interrupt is safe for
        // same-direction retargets; reverse always went through FOLLOW_STOP.
        const bool running = (_status_word & STATUS_RUNNING) != 0;
        const bool in_pos = (_status_word & STATUS_IN_POS) != 0;
        const uint16_t motion = (running && !in_pos && !_follow_via_stop) ? MOTION_ABS_IRQ : MOTION_ABS;
        send_u16(REG_MOTION, motion);
        break;
    }
    case Phase::MID_ZERO:
        send_u16(REG_AUX, AUX_POS_ZERO);
        break;
    case Phase::SEEK_POLL:
        // Bias toward DI, but keep encoder/status fresh (stall detect needs motion).
        switch (_poll_step % 4) {
        case 0:
        case 2:
            send_read(REG_DI, 1, RxExpect::DI);
            break;
        case 1:
            send_read(REG_ENCODER, 2, RxExpect::ENCODER);
            break;
        default:
            send_read(REG_STATUS, 2, RxExpect::STATUS);
            break;
        }
        break;
    case Phase::BACK_POLL:
    case Phase::MID_POLL:
    case Phase::IDLE:
        switch (_poll_step % 3) {
        case 0:
            send_read(REG_ENCODER, 2, RxExpect::ENCODER);
            break;
        case 1:
            send_read(REG_STATUS, 2, RxExpect::STATUS);
            break;
        default:
            send_read(REG_DI, 1, RxExpect::DI);
            break;
        }
        break;
    }
}

void AP_ModbusSteering::update(float steering_out)
{
    _stick = constrain_float(steering_out, -1.0f, 1.0f);
    if (_uart == nullptr || !_uart->is_initialized()) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    consume_rx();
    poll_triggers();

    if (hal.util->get_soft_armed() && seeking()) {
        abort_home("armed during home");
        return;
    }

    if (_waiting) {
        if (take_reply()) {
            _waiting = false;
            _tx_tries = 0;
            on_reply(now);
        } else if (_wait_ms != 0 && (now - _wait_ms) > REPLY_TIMEOUT_MS) {
            _waiting = false;
            _tx_tries++;
            if (_tx_tries >= TX_TRIES) {
                _tx_tries = 0;
                if (seeking()) {
                    abort_home("no Modbus reply");
                } else if (following()) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: follow lost reply");
                    _follow_profile_ok = false;
                    enter(Phase::IDLE);
                } else if (_phase == Phase::ABORT_STOP || _phase == Phase::ALARM_CLEAR ||
                           _phase == Phase::ALARM_ENABLE) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: stop failed, no reply");
                    enter(Phase::IDLE);
                } else if (_last_init_warn_ms == 0 || (now - _last_init_warn_ms) > 5000U) {
                    _last_init_warn_ms = now;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: no reply during init");
                }
            }
        }
        return;
    }

    if (_last_send_ms != 0 && (now - _last_send_ms) < SEND_GAP_MS) {
        return;
    }
    if (_phase == Phase::IDLE) {
        if (try_begin_follow()) {
            send_current();
            return;
        }
        _poll_step++;
    }
    send_current();
}
