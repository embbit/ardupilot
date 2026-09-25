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
constexpr uint16_t REG_DI_STATE = 0x0005;  // X0..X6 raw inputs
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
constexpr uint16_t REG_TRACK_ERR = 0x0052;
constexpr uint16_t AUX_ALARM_CLEAR = 0x0004;
constexpr uint16_t AUX_POS_ZERO = 0x0008;
// CL57R 0x0036 (official Modbus doc):
// Bit0 position start, Bit1 abs/rel, Bit2 irq, Bit3 speed start,
// Bit4 home, Bit5 stop, Bit6 e-stop.
// After native home the drive often ignores Bit0 until Bit5 stop exits home mode.
constexpr uint16_t MOTION_START_REL = 0x0001;
constexpr uint16_t MOTION_START_ABS = 0x0003;
constexpr uint16_t MOTION_START_ABS_IRQ = 0x0007;
constexpr uint16_t MOTION_SPEED = 0x0008;
constexpr uint16_t MOTION_HOME = 0x0010;
constexpr uint16_t MOTION_STOP = 0x0020;
constexpr uint16_t MOTION_ESTOP = 0x0040;
constexpr uint16_t STATUS_HOME_DONE = (1U << 1);
constexpr uint16_t STATUS_RUNNING = (1U << 2);
// CL57R DI 0x0005: X1=P-OT (bit1), X2=N-OT (bit2) with default DI function map.
constexpr uint16_t DI_BIT_POT = (1U << 1);
constexpr uint16_t DI_BIT_NOT = (1U << 2);
constexpr uint16_t SUBDIVISION_PPR = 4000;
constexpr uint16_t ACCEL_DEFAULT = 200;
constexpr uint16_t DECEL_DEFAULT = 200;
constexpr uint16_t MID_SEEK_ACCEL_MS = 800;
constexpr uint16_t MID_SEEK_DECEL_MS = 800;
constexpr uint32_t SEND_INTERVAL_MS = 50;
constexpr uint32_t BUTTON_LOCKOUT_MS = 300;
constexpr uint32_t HOME_TIMEOUT_MS = 90000;
constexpr uint32_t WAIT_ECHO_WARN_MS = 5000;
constexpr uint32_t HOME_RX_LOST_WARN_MS = 15000;
constexpr uint32_t HOME_RX_ABORT_MS = 30000;
constexpr uint32_t HOME_RX_SILENCE_MS = 12000;
constexpr int32_t MIN_MEASURED_TRAVEL = 1000;
constexpr int32_t MIN_HOME_LEG_MOTION = 2000;
}

const AP_Param::GroupInfo AP_ModbusSteering::var_info[] = {
    // @Param: SLAVE_ID
    // @DisplayName: Modbus Slave ID
    // @Description: CL57R Modbus slave address
    // @Range: 1 247
    // @User: Standard
    AP_GROUPINFO("SLAVE_ID", 1, AP_ModbusSteering, slave_id, 1),

    // @Param: REG_ADDR
    // @DisplayName: Unused (legacy)
    // @Description: Unused. Kept so existing parameter storage indices stay valid.
    // @User: Advanced
    AP_GROUPINFO("REG_ADDR", 2, AP_ModbusSteering, reg_address, 52),

    // @Param: MAX_STEPS
    // @DisplayName: Maximum Steering Steps
    // @Description: Pulses at full stick (+/-1) when OUT_REV=0. Otherwise OUT_REV*RATIO*4000/2.
    // @User: Standard
    AP_GROUPINFO("MAX_STEPS", 3, AP_ModbusSteering, max_steps, 200000),

    // @Param: START_SPD
    // @DisplayName: Positioning start speed
    // @Description: CL57R trapezoid start speed (register 0x0030) for positioning/speed moves after init
    // @Units: RPM
    // @Range: 2 300
    // @User: Standard
    AP_GROUPINFO("START_SPD", 4, AP_ModbusSteering, start_speed, 15),

    // @Param: MAX_SPD
    // @DisplayName: Armed run speed
    // @Description: CL57R max speed (register 0x0033) while armed / normal stick steering after calibration
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

    // @Param: TRACK_ERR
    // @DisplayName: Tracking error limit
    // @Description: CL57R register 0x0052 max encoder following error (pulses) before alarm. Too small causes false limit hits under load; too large delays hard-stop detection. Do not use 65535 (illegal on many CL57R). Try 8000-20000 if home trips early when reversing off a stop.
    // @Units: pulses
    // @Range: 100 30000
    // @User: Advanced
    AP_GROUPINFO("TRACK_ERR", 7, AP_ModbusSteering, track_err, 10000),

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
    // @DisplayName: CL57R first home method
    // @Description: First limit for homing. 17 searches the negative limit (X2 N-OT). 18 searches the positive limit (X1 P-OT). In DualLimit mode the driver homes to the opposite limit second and measures travel.
    // @Values: 17:NegativeLimit,18:PositiveLimit
    // @Range: 17 18
    // @User: Standard
    AP_GROUPINFO("HOME_MTH", 12, AP_ModbusSteering, home_mth, 17),

    // @Param: HOME_SPD
    // @DisplayName: Homing approach speed
    // @Description: CL57R fast approach speed during calibration (register 0x0041). Crawl near the limit is separate (0x0042, capped at 300 RPM). Also used as the calib MAX_SPD write; mid-seek to center uses a slower crawl-range speed.
    // @Units: RPM
    // @Range: 5 3000
    // @User: Standard
    AP_GROUPINFO("HOME_SPD", 13, AP_ModbusSteering, home_speed, 1800),

    // @Param: HOME_TRIG
    // @DisplayName: Homing trigger
    // @Description: Start CL57R actions from the GCS. Set to 1 to start calibration (alarm clear, home, center, zero). Set to 2 to clear the alarm only. Ignored while armed. Resets to 0 when the action completes or is rejected.
    // @Values: 0:None,1:Calibrate,2:ClearAlarm
    // @User: Standard
    AP_GROUPINFO("HOME_TRIG", 14, AP_ModbusSteering, home_trig, 0),

    // @Param: HOME_MODE
    // @DisplayName: Homing mode
    // @Description: 0 uses one limit switch and OUT_REV/RATIO for center and travel. 1 homes to both limits, measures encoder travel lock-to-lock, centers at the midpoint, and saves half-travel to MAX_STEPS.
    // @Values: 0:SingleLimit,1:DualLimit
    // @User: Standard
    AP_GROUPINFO("HOME_MODE", 15, AP_ModbusSteering, home_mode, 1),

    // @Param: JOG
    // @DisplayName: Manual jog trigger
    // @Description: Disarmed recovery jog. Set 1 to step positive, 2 to step negative. Uses JOG_PUL pulses at crawl RPM. Clears alarm and reissues speed if latched. A new JOG while busy preempts the previous step.
    // @Values: 0:None,1:JogPositive,2:JogNegative
    // @User: Standard
    AP_GROUPINFO("JOG", 16, AP_ModbusSteering, jog_trig, 0),

    // @Param: JOG_PUL
    // @DisplayName: Jog step pulses
    // @Description: Encoder pulses moved per JOG trigger. 4000 = one motor revolution.
    // @Units: pulses
    // @Range: 100 50000
    // @User: Standard
    AP_GROUPINFO("JOG_PUL", 17, AP_ModbusSteering, jog_pulses, 2000),

    // @Param: LIM_INV
    // @DisplayName: Limit input invert
    // @Description: 0 = limit active when DI bit is 1 (NO). 1 = active when DI bit is 0 (NC). Applies to both P-OT (X1) and N-OT (X2) on register 0x0005.
    // @Values: 0:ActiveHigh,1:ActiveLow
    // @User: Advanced
    AP_GROUPINFO("LIM_INV", 18, AP_ModbusSteering, lim_inv, 0),

    // @Param: LIM_EN
    // @DisplayName: Enforce DI limit blocking
    // @Description: 0 = ignore P-OT/N-OT for motion blocking (DI still logged). 1 = refuse seek/follow into an active limit. JOG is never hard-blocked so you can recover. Enable only after DI bits look correct in the middle (both inactive).
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO("LIM_EN", 19, AP_ModbusSteering, lim_en, 0),

    AP_GROUPEND
};

AP_ModbusSteering::AP_ModbusSteering()
{
    AP_Param::setup_object_defaults(this, var_info);
}

int32_t AP_ModbusSteering::travel_limit_pulses() const
{
    if (_measured_half_travel > 0) {
        return _measured_half_travel;
    }
    if (out_rev.get() > 0 && ratio.get() > 0) {
        return (int32_t)out_rev.get() * ratio.get() * CL57R_STEPS_PER_REV / 2;
    }
    return max_steps.get();
}

int32_t AP_ModbusSteering::expected_full_travel_pulses() const
{
    if (out_rev.get() > 0 && ratio.get() > 0) {
        return (int32_t)out_rev.get() * ratio.get() * CL57R_STEPS_PER_REV;
    }
    return max_steps.get() * 2;
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
           _state == DriveState::HOME_ZERO || _state == DriveState::HOME_ZERO_AT_L1;
}

bool AP_ModbusSteering::dual_limit_home() const
{
    return home_mode.get() == 1;
}

uint16_t AP_ModbusSteering::home_first_method() const
{
    return home_mth.get() == 18 ? 18 : 17;
}

uint16_t AP_ModbusSteering::home_method_reg() const
{
    const uint16_t first = home_first_method();
    if (dual_limit_home() && _home_leg == 1) {
        return first == 17 ? 18 : 17;
    }
    return first;
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
    if (rpm < 5) {
        return 1800;
    }
    if (rpm > 3000) {
        return 3000;
    }
    return (uint16_t)rpm;
}

uint16_t AP_ModbusSteering::calib_crawl_rpm() const
{
    // 0x0042 is limited to 5..300. Dual-limit legs on this drive often run the
    // whole travel at crawl (limit already seen / polarity) — use the max.
    const uint16_t home = calib_speed_rpm();
    if (home >= 300) {
        return 300;
    }
    if (home < 5) {
        return 30;
    }
    return home;
}

uint16_t AP_ModbusSteering::mid_seek_speed_rpm() const
{
    // Same cruise as dual-limit speed-mode seek (OB_STR_HOME_SPD).
    return calib_speed_rpm();
}

uint16_t AP_ModbusSteering::track_err_limit() const
{
    // CL57R rejects 0xFFFF (illegal data). Keep a safe usable window.
    int32_t v = track_err.get();
    if (v < 100) {
        return 100;
    }
    if (v > 30000) {
        return 30000;
    }
    return (uint16_t)v;
}

uint16_t AP_ModbusSteering::seek_track_err_limit() const
{
    // Dual-limit seek must tolerate lag at HOME_SPD. A tight user TRACK_ERR
    // (e.g. 4000) trips mid-span and falsely ends a leg. Boost for seek only.
    const uint16_t user = track_err_limit();
    const uint16_t seek_min = 25000;
    return (user > seek_min) ? user : seek_min;
}

bool AP_ModbusSteering::lim_pos_active() const
{
    if (!_got_di) {
        return false;
    }
    const bool bit = (_di_state & DI_BIT_POT) != 0;
    return lim_inv.get() != 0 ? !bit : bit;
}

bool AP_ModbusSteering::lim_neg_active() const
{
    if (!_got_di) {
        return false;
    }
    const bool bit = (_di_state & DI_BIT_NOT) != 0;
    return lim_inv.get() != 0 ? !bit : bit;
}

bool AP_ModbusSteering::dir_blocked(int8_t sign) const
{
    if (lim_en.get() == 0) {
        return false;
    }
    // Both bits active ⇒ floating/unused DI, not real limits — ignore.
    if (lim_pos_active() && lim_neg_active()) {
        return false;
    }
    if (sign > 0) {
        return lim_pos_active();
    }
    if (sign < 0) {
        return lim_neg_active();
    }
    return false;
}

int32_t AP_ModbusSteering::center_target_pulses() const
{
    const int32_t limit = travel_limit_pulses();
    // After finishing on method M17 the free mid lies in the negative
    // command direction; after M18 it is positive (matches measured leg sign).
    if (home_method_reg() == 18) {
        return limit;
    }
    return -limit;
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
        const int8_t port = serial_manager.find_portnum((AP_SerialManager::SerialProtocol)101, 0);
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
}

void AP_ModbusSteering::send_target_pos(int32_t target)
{
    uint8_t tx_packet[16];
    uint16_t values[2];
    values[0] = (uint16_t)((target >> 16) & 0xFFFF);
    values[1] = (uint16_t)(target & 0xFFFF);
    modbus_create_write_multiple_packet((uint8_t)slave_id.get(), REG_TARGET_POS, 2, values, tx_packet);
    _uart->write(tx_packet, 13);
    _last_target = target;
    _have_target = true;
}

void AP_ModbusSteering::queue_motion(uint16_t motion)
{
    _queued_motion = motion;
}

bool AP_ModbusSteering::flush_queued_motion()
{
    if (_queued_motion == 0) {
        return false;
    }
    send_u16(REG_MOTION, _queued_motion);
    _queued_motion = 0;
    return true;
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
    _alarm_clear_pending = true;
    _enable_after_alarm_clear = true;
    _have_target = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: alarm clear");
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
        if (!_home_pending) {
            _home_pending = true;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home queued");
        }
        return;
    }
    _alarm_clear_pending = false;
    _homed = false;
    _got_echo = false;
    _init_attempts = 0;
    _got_status = false;
    _saw_home_run = false;
    _saw_home_motion = false;
    _saw_home_clear = false;
    _home_center_run_spd = false;
    _home_leg_settling = false;
    _center_step_settling = false;
    _home_leg = 0;
    _measured_half_travel = 0;
    _leg_peak_travel = 0;
    _leg_dir_sign = 0;
    _center_move_target = 0;
    _steer_cmd_offset = 0;
    _center_resend = false;
    _speed_follow = false;
    _follow_prep = 0;
    _follow_moving = false;
    _follow_sign = 0;
    _follow_slot = 0;
    _follow_alarm_step = 0;
    _follow_restore = 0;
    _follow_last_enc = 0;
    _follow_last_spd = 0;
    _follow_progress_ms = 0;
    _follow_mid_retried = false;
    _follow_halted = false;
    _follow_alarm_count = 0;
    _follow_peak_toward = 0;
    _queued_motion = 0;
    _home_retry_pending = false;
    _home_stop_pending = false;
    _home_clear_pending = false;
    _home_speed_leg = false;
    _home_crawl_pending = false;
    _home_crawl_away = false;
    _home_early_retries = 0;
    _home_early_peak = 0;
    _leg1_travel = 0;
    _state = DriveState::HOME_CLEAR_ALARM;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home start");
}

void AP_ModbusSteering::poll_param_trigger()
{
    const int8_t trig = home_trig.get();
    if (!_home_trig_inited) {
        _home_trig_last = trig;
        _home_trig_inited = true;
        return;
    }
    if (trig == 0) {
        _home_trig_last = 0;
        return;
    }

    if (hal.util->get_soft_armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: HOME_TRIG ignored, armed");
        home_trig.set_and_save(0);
        _home_trig_last = 0;
        return;
    }

    if (trig == _home_trig_last) {
        return;
    }
    _home_trig_last = trig;

    if (trig == 1) {
        if (in_home()) {
            abort_home("restart home");
        }
        start_home();
    } else if (trig == 2) {
        if (in_home()) {
            abort_home("alarm clear");
        }
        request_alarm_clear();
        home_trig.set_and_save(0);
        _home_trig_last = 0;
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: invalid HOME_TRIG %d", (int)trig);
        home_trig.set_and_save(0);
        _home_trig_last = 0;
    }
}

void AP_ModbusSteering::start_jog(int8_t sign)
{
    if (sign == 0) {
        return;
    }
    // Never hard-block JOG on DI — false P-OT/N-OT (floating inputs) must not
    // trap the rudder. Warn only so the operator can set LIM_INV / wiring.
    if (_got_di && ((sign > 0 && lim_pos_active()) || (sign < 0 && lim_neg_active()))) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "CL57R: DI=0x%02x P=%u N=%u (jog anyway)",
                      (unsigned)(_di_state & 0x7F),
                      (unsigned)lim_pos_active(),
                      (unsigned)lim_neg_active());
    } else if (_got_di) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                      "CL57R: DI=0x%02x P=%u N=%u",
                      (unsigned)(_di_state & 0x7F),
                      (unsigned)lim_pos_active(),
                      (unsigned)lim_neg_active());
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: jog, DI not read yet");
    }
    int32_t need = jog_pulses.get();
    if (need < 100) {
        need = 100;
    }
    if (need > 50000) {
        need = 50000;
    }
    // Pause stick follow so mid-offset cannot fight the recovery jog.
    _follow_moving = false;
    _follow_sign = 0;
    _follow_slot = 0;
    _follow_halted = false;
    _steer_cmd_offset = 0;
    _jog_active = true;
    _jog_sign = sign;
    // Slot sequence: STOP → CLEAR → ENABLE → SPD → SPEED → run
    _jog_slot = 0;
    _jog_start_enc = _actual_pulses;
    _jog_need = need;
    _jog_start_ms = AP_HAL::millis();
    _jog_last_progress_ms = _jog_start_ms;
    _alarm_clear_pending = false;
    _enable_after_alarm_clear = false;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: jog %s %d pul",
                  sign > 0 ? "+" : "-", (int)need);
}

void AP_ModbusSteering::poll_jog_trigger()
{
    const int8_t trig = jog_trig.get();
    if (!_jog_trig_inited) {
        _jog_trig_last = trig;
        _jog_trig_inited = true;
        return;
    }
    if (trig == 0) {
        _jog_trig_last = 0;
        return;
    }
    if (hal.util->get_soft_armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: JOG ignored, armed");
        jog_trig.set_and_save(0);
        _jog_trig_last = 0;
        return;
    }
    if (trig == _jog_trig_last) {
        return;
    }
    _jog_trig_last = trig;

    if (in_home()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: JOG ignored, homing");
        jog_trig.set_and_save(0);
        _jog_trig_last = 0;
        return;
    }
    // Preempt a stuck jog (latched alarm / no encoder motion).
    if (_jog_active) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: JOG preempt");
    }

    if (trig == 1) {
        start_jog(1);
    } else if (trig == 2) {
        start_jog(-1);
    } else {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: invalid JOG %d", (int)trig);
    }
    jog_trig.set_and_save(0);
    _jog_trig_last = 0;
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

void AP_ModbusSteering::home_leg_done(uint32_t now)
{
    (void)now;
    if (dual_limit_home() && _home_leg == 0) {
        _leg1_travel = _leg_peak_travel;
        _home_leg = 1;
        _home_early_retries = 0;
        _home_early_peak = 0;
        _home_crawl_pending = false;
        _state = DriveState::HOME_ZERO_AT_L1;
        _got_echo = false;
        _init_attempts = 0;
        _home_stop_pending = false;
        _home_clear_pending = false;
        _home_leg_settling = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: limit 1 reached, zero and seek limit 2");
        return;
    }

    if (dual_limit_home()) {
        // CL57R native home typically zeros the encoder at the limit, so the
        // final reading after leg 2 is near 0. Use peak displacement tracked
        // while seeking the second limit.
        int32_t full_travel = _leg_peak_travel;
        const int32_t abs_now = (_actual_pulses >= 0) ? _actual_pulses : -_actual_pulses;
        if (abs_now > full_travel) {
            full_travel = abs_now;
        }
        const int32_t expected = expected_full_travel_pulses();
        // Leg2 sometimes trips tracking when backing out of L1 (travel ~4k).
        // If leg1 already measured a plausible lock-to-lock, use that.
        if (expected >= 20000 && full_travel < expected / 4 &&
            _leg1_travel >= expected / 4 && _leg1_travel <= expected * 5) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "CL57R: leg2 short %d, use leg1 %d",
                          (int)full_travel, (int)_leg1_travel);
            full_travel = _leg1_travel;
            // Still sitting near L1 — mid is away from L1 (= leg2 seek direction).
            if (_leg_dir_sign == 0) {
                _leg_dir_sign = (home_method_reg() == 18) ? 1 : -1;
            }
            // Invert usual mid_sign logic below: we want SAME as leg2 dir.
            _leg_dir_sign = (int8_t)(-_leg_dir_sign);
        }
        if (full_travel < MIN_MEASURED_TRAVEL) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: travel %d pulses (min %d)",
                          (int)full_travel, (int)MIN_MEASURED_TRAVEL);
            abort_home("measured travel too small (need both limits?)");
            return;
        }
        // Reject / clamp oversize. A late tracking alarm after overshooting the
        // second switch (into the hard stop) can report ~2x true travel; using
        // half of that as mid drives INTO the opposite limit.
        if (expected >= 20000 && full_travel > (expected + expected / 4)) {
            int32_t use = expected;
            if (_leg1_travel >= (expected * 3) / 4 &&
                _leg1_travel <= (expected + expected / 4)) {
                use = _leg1_travel;
            }
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "CL57R: travel %d oversize, use %d",
                          (int)full_travel, (int)use);
            full_travel = use;
        }
        if (expected >= 20000 && full_travel > expected * 5) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: got %d, expected ~%d",
                          (int)full_travel, (int)expected);
            abort_home("measured travel implausible");
            return;
        }
        if (expected >= 20000 && full_travel < (expected * 5) / 8) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: got %d, expected ~%d",
                          (int)full_travel, (int)expected);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "CL57R: raise TRACK_ERR (false early limit?)");
            abort_home("measured travel implausible");
            return;
        }
        _measured_half_travel = full_travel / 2;
        // After leg 2 we sit on that limit (encoder ~0). Mid is BACK toward
        // the other limit — opposite of the signed travel measured on leg 2.
        // User log: M17 as leg 2 + positive cmd slammed the stop; leg sign fixes it.
        int8_t mid_sign = 0;
        if (_leg_dir_sign > 0) {
            mid_sign = -1;
        } else if (_leg_dir_sign < 0) {
            mid_sign = 1;
        } else {
            // Fallback if encoder never moved (should not happen after a valid leg).
            mid_sign = (home_method_reg() == 18) ? 1 : -1;
        }
        _center_move_target = (int32_t)mid_sign * _measured_half_travel;
        // Never command more than ~expected/2 for mid — hard safety cap.
        if (expected >= 20000) {
            const int32_t half_exp = expected / 2;
            const int32_t abs_mid = (_center_move_target >= 0) ?
                                   _center_move_target : -_center_move_target;
            if (abs_mid > half_exp + half_exp / 8) {
                _center_move_target = (int32_t)mid_sign * half_exp;
                _measured_half_travel = half_exp;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: mid clamp to %d", (int)_center_move_target);
            }
        }
        max_steps.set_and_save(_measured_half_travel);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: travel %d mid %d (legdir %d)",
                      (int)full_travel, (int)_center_move_target, (int)_leg_dir_sign);
    } else {
        _center_move_target = center_target_pulses();
    }

    // Absolute moves after native home are ignored by this CL57R. Use continuous
    // speed-mode follow toward stick + mid offset (stick=0 returns to center).
    _center_encoder_origin = _actual_pulses;
    _steer_cmd_offset = _center_move_target;
    _speed_follow = true;
    _follow_prep = 0;
    _follow_moving = false;
    _follow_sign = 0;
    _follow_slot = 0;
    _follow_alarm_step = 0;
    _follow_restore = 0;
    _follow_last_enc = 0;
    _follow_last_spd = 0;
    _follow_progress_ms = 0;
    _follow_mid_retried = false;
    _follow_halted = false;
    _follow_alarm_count = 0;
    _follow_peak_toward = 0;
    _home_start_ms = AP_HAL::millis();
    _last_home_progress_ms = 0;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "CL57R: cal@limit ofs %d v17",
                  (int)_steer_cmd_offset);
    finish_home();
}

void AP_ModbusSteering::finish_home()
{
    _state = DriveState::RUN_WRITE;
    _have_target = false;
    _last_target = 0;
    _homed = true;
    // Mid return uses gentle crawl RPM first; restore run MAX_SPD after arrive.
    if (!_speed_follow) {
        _pending_run_spd = true;
    }
    _pending_track_restore = true;
    _alarm_clear_pending = true;
    _enable_after_alarm_clear = true;
    _rx_expect = RxExpect::NONE;
    if (home_trig.get() == 1) {
        home_trig.set_and_save(0);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: calibrated");
}

void AP_ModbusSteering::abort_home(const char *reason)
{
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: %s", reason);
    _state = DriveState::RUN_WRITE;
    _have_target = false;
    _rx_expect = RxExpect::NONE;
    _pending_track_restore = true;
    _home_stop_pending = false;
    _home_clear_pending = false;
    _home_crawl_pending = false;
    if (home_trig.get() == 1) {
        home_trig.set_and_save(0);
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
        const uint16_t received_crc = (_rx_buf[offset + frame_len - 1] << 8) | _rx_buf[offset + frame_len - 2];
        if (modbus_crc16(&_rx_buf[offset], frame_len - 2) != received_crc) {
            offset++;
            continue;
        }

        _ever_got_rx = true;
        _last_rx_ms = AP_HAL::millis();

        const uint8_t fn = _rx_buf[offset + 1];
        if (fn == 0x06) {
            _got_echo = true;
        } else if (fn == 0x10) {
            // FC16 echo (write-multiple) also counts as a successful exchange.
            _got_echo = true;
        } else if ((fn & 0x80) != 0) {
            // Exception response still proves the slave heard us; advance so an
            // unsupported register cannot stall homing forever.
            _got_echo = true;
            if (frame_len >= 3) {
                static uint32_t last_ex_ms;
                const uint32_t tnow = AP_HAL::millis();
                if (last_ex_ms == 0 || (tnow - last_ex_ms) > 2000) {
                    last_ex_ms = tnow;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                  "CL57R: Modbus ex fn=0x%02x code=%u",
                                  (unsigned)fn, (unsigned)_rx_buf[offset + 2]);
                }
            }
        } else if (fn == 0x03 && frame_len >= 7) {
            const uint8_t byte_count = _rx_buf[offset + 2];
            if (_rx_expect == RxExpect::STATUS && byte_count >= 2) {
                _status_word = ((uint16_t)_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4];
                _got_status = true;
            } else if (_rx_expect == RxExpect::ENCODER && byte_count == 0x04 && frame_len >= 9) {
                const uint16_t high_word = (_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4];
                const uint16_t low_word = (_rx_buf[offset + 5] << 8) | _rx_buf[offset + 6];
                _actual_pulses = (int32_t)(((uint32_t)high_word << 16) | low_word);
            } else if (_rx_expect == RxExpect::DI && byte_count >= 2) {
                _di_state = ((uint16_t)_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4];
                _got_di = true;
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
        _state = DriveState::INIT_TRACK_ERR;
        break;
    case DriveState::INIT_TRACK_ERR:
        _state = DriveState::RUN_WRITE;
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
        _state = DriveState::HOME_ENABLE;
        break;
    case DriveState::HOME_ENABLE:
        // Dual-limit: drive toward the stop in speed mode so HOME_SPD is used.
        // Native MOTION_HOME on this CL57R often crawls the whole travel at
        // HOME_CRAWL (<=300) when the limit input is already seen.
        if (dual_limit_home()) {
            _home_speed_leg = true;
            _state = DriveState::HOME_SET_TRACK_ERR;
        } else {
            _home_speed_leg = false;
            _state = DriveState::HOME_START;
        }
        break;
    case DriveState::HOME_SET_TRACK_ERR:
        _state = DriveState::HOME_SEEK_SPD;
        break;
    case DriveState::HOME_SEEK_SPD:
        _state = DriveState::HOME_START;
        break;
    case DriveState::HOME_START:
        _state = DriveState::HOME_WAIT;
        _home_start_ms = AP_HAL::millis();
        _last_home_retry_ms = _home_start_ms;
        _last_home_progress_ms = _home_start_ms;
        _home_start_pulses = _actual_pulses;
        _leg_peak_travel = 0;
        _leg_dir_sign = 0;
        _home_read_encoder = false;
        _got_status = false;
        _saw_home_run = false;
        _saw_home_motion = false;
        _saw_home_clear = false;
        _home_leg_settling = false;
        _home_stop_pending = false;
        _home_clear_pending = false;
        _home_crawl_pending = false;
        _home_crawl_away = false;
        _home_early_retries = 0;
        _home_early_peak = 0;
        _home_ignore_alarm_ms = 0;
        if (_home_speed_leg) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: limit seek leg %u @%urpm",
                          (unsigned)(_home_leg + 1),
                          (unsigned)calib_speed_rpm());
        } else if (dual_limit_home()) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: homing leg %u (M%d)",
                          (unsigned)(_home_leg + 1), (int)home_method_reg());
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: homing to limit (M%d)", (int)home_method_reg());
        }
        break;
    case DriveState::HOME_ZERO_AT_L1:
        _state = DriveState::HOME_SET_METHOD;
        _got_echo = false;
        _init_attempts = 0;
        _saw_home_motion = false;
        _saw_home_run = false;
        _saw_home_clear = false;
        _home_start_ms = AP_HAL::millis();
        _last_home_progress_ms = _home_start_ms;
        break;
    case DriveState::HOME_ZERO:
        _steer_cmd_offset = 0;
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
    poll_param_trigger();
    poll_jog_trigger();
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

    if (in_home()) {
        if (!_ever_got_rx) {
            if (_last_home_norx_ms == 0 || (now - _last_home_norx_ms) >= HOME_RX_LOST_WARN_MS) {
                _last_home_norx_ms = now;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                              "CL57R: no Modbus RX from CL57R (check RS485 A/B RX)");
            }
        } else if (_last_rx_ms == 0 || (now - _last_rx_ms) > HOME_RX_SILENCE_MS) {
            if (_home_rx_lost_ms == 0) {
                _home_rx_lost_ms = now;
            }
            // Only abort if we never saw motion — long home legs can go quiet while
            // the drive is busy seeking a limit. Do not abort mid-travel on silence.
            if ((now - _home_rx_lost_ms) >= HOME_RX_ABORT_MS &&
                !_saw_home_motion &&
                (_state == DriveState::HOME_WAIT || _state == DriveState::HOME_WAIT_CENTER)) {
                abort_home("Modbus RX lost, abort home");
            } else if (!_saw_home_motion &&
                       (now - _home_rx_lost_ms) >= HOME_RX_ABORT_MS &&
                       (_last_home_norx_ms == 0 ||
                        (now - _last_home_norx_ms) >= HOME_RX_LOST_WARN_MS)) {
                // Only nag when we never saw motion and are near abort.
                _last_home_norx_ms = now;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: Modbus RX quiet during home");
            }
        } else {
            _home_rx_lost_ms = 0;
        }
    } else {
        _home_rx_lost_ms = 0;
    }

    if (_state == DriveState::HOME_WAIT) {
        const bool home_bit = (_got_status && (_status_word & STATUS_HOME_DONE) != 0);
        const bool running = (_got_status && (_status_word & STATUS_RUNNING) != 0);
        const int32_t delta = _actual_pulses - _home_start_pulses;
        const int32_t moved = (delta >= 0) ? delta : -delta;
        if (moved > _leg_peak_travel) {
            _leg_peak_travel = moved;
            _last_home_progress_ms = now;
            if (delta > 0) {
                _leg_dir_sign = 1;
            } else if (delta < 0) {
                _leg_dir_sign = -1;
            }
        }
        if (_got_status && !home_bit) {
            _saw_home_clear = true;
        }
        if (moved >= MIN_HOME_LEG_MOTION) {
            _saw_home_motion = true;
        }
        if (running && moved >= 50) {
            _saw_home_run = true;
        }
        if (now - _home_start_ms > HOME_TIMEOUT_MS) {
            abort_home("home timeout");
        } else if (_home_speed_leg) {
            // Speed-mode limit seek: stop on alarm/stall, but treat a short
            // "hit" as tracking fault (retry crawl) — not a real endstop.
            // Do NOT finish a leg on DI alone — floating X1/X2 bits caused
            // false "DI limit hit" with ~7k travel and aborted calibration.
            const int32_t expected = expected_full_travel_pulses();
            // Leg1 often starts near mid (~expected/2 to a stop). Leg2 is
            // lock-to-lock (~expected). Low TRACK_ERR false hits are shorter.
            const int32_t min_real = (expected >= 20000)
                ? ((_home_leg == 0) ? ((expected * 2) / 5) : ((expected * 5) / 8))
                : 20000;
            const bool alarm_now = _got_status && alarmed() &&
                                   (now >= _home_ignore_alarm_ms);
            const bool hit = _saw_home_motion && alarm_now;
            const bool stalled = _saw_home_motion &&
                                 (now - _last_home_progress_ms) > 1500 &&
                                 !running;
            // Soft max travel: if we have already moved past OUT_REV estimate,
            // stop — do not grind into the hard stop waiting for a late alarm.
            const bool past_expected =
                (expected >= 20000) &&
                _saw_home_motion &&
                (_leg_peak_travel >= expected + expected / 8);
            if ((hit || stalled || past_expected) &&
                !_home_stop_pending && !_home_clear_pending && !_home_crawl_pending) {
                // Crawl made no further progress ⇒ jammed on a real stop/switch
                // (started near that limit). Do not abort as "early only".
                const bool stuck_on_stop =
                    (_home_early_retries >= 1) &&
                    (_leg_peak_travel >= 5000) &&
                    (_leg_peak_travel <= _home_early_peak + 2000);
                if (past_expected && !hit && !stalled) {
                    if (!_home_leg_settling) {
                        _home_leg_settling = true;
                        _home_leg_settle_ms = now + 250;
                        _home_stop_pending = true;
                        _home_crawl_pending = false;
                        _home_crawl_away = false;
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                      "CL57R: seek past expected %d, stop",
                                      (int)_leg_peak_travel);
                    } else if (now >= _home_leg_settle_ms) {
                        _home_leg_settling = false;
                        home_leg_done(now);
                    }
                } else if (_leg_peak_travel < min_real && !stuck_on_stop &&
                           _home_early_retries < 6) {
                    _home_early_peak = _leg_peak_travel;
                    _home_early_retries++;
                    _home_leg_settling = false;
                    _home_stop_pending = true;
                    _home_crawl_pending = true;
                    _home_crawl_away = false;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                  "CL57R: early alarm %d, crawl retry %u",
                                  (int)_leg_peak_travel,
                                  (unsigned)_home_early_retries);
                } else if (_leg_peak_travel < min_real && !stuck_on_stop) {
                    abort_home("early limits only; raise TRACK_ERR / check mech");
                } else if (!_home_leg_settling) {
                    _home_leg_settling = true;
                    _home_leg_settle_ms = now + 250;
                    _home_stop_pending = true;
                    _home_crawl_pending = false;
                    _home_crawl_away = false;
                    if (stuck_on_stop && _leg_peak_travel < min_real) {
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                      "CL57R: limit hit (stuck) %d",
                                      (int)_leg_peak_travel);
                    } else {
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                      "CL57R: limit hit travel %d",
                                      (int)_leg_peak_travel);
                    }
                } else if (now >= _home_leg_settle_ms) {
                    _home_leg_settling = false;
                    home_leg_done(now);
                }
            } else if (_home_leg_settling &&
                       !_home_stop_pending && !_home_clear_pending &&
                       !_home_crawl_pending &&
                       now >= _home_leg_settle_ms) {
                _home_leg_settling = false;
                home_leg_done(now);
            } else if (!_saw_home_motion && (now - _last_home_progress_ms) > 10000) {
                _last_home_progress_ms = now;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: limit seek, no motion yet");
            }
        } else if (home_bit && !running && !_saw_home_motion &&
                   (now - _home_start_ms) > 5000) {
            if (now - _last_home_retry_ms > 3000) {
                _last_home_retry_ms = now;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: home done bit set, no motion, retry");
            }
        } else if (!_saw_home_motion && (now - _last_home_progress_ms) > 10000) {
            _last_home_progress_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home searching, no motion yet");
        } else if (home_bit && !running && _saw_home_clear && _saw_home_motion) {
            if (!_home_leg_settling) {
                _home_leg_settling = true;
                _home_leg_settle_ms = now + 200;
            } else if (now >= _home_leg_settle_ms) {
                _home_leg_settling = false;
                home_leg_done(now);
            }
        } else if (_home_leg_settling && running) {
            _home_leg_settling = false;
        }
    } else if (_state == DriveState::HOME_WAIT_CENTER) {
        const int32_t enc_from_origin = _actual_pulses - _center_encoder_origin;
        const int32_t step_err = (enc_from_origin > _center_step_target) ?
                                 (enc_from_origin - _center_step_target) :
                                 (_center_step_target - enc_from_origin);
        const int32_t final_err = (enc_from_origin > _center_move_target) ?
                                  (enc_from_origin - _center_move_target) :
                                  (_center_move_target - enc_from_origin);
        int32_t arrive = pos_db.get();
        if (arrive < 500) {
            arrive = 500;
        }
        const int32_t cap = travel_limit_pulses() / 40;
        if (cap >= 500 && arrive > cap) {
            arrive = cap;
        }
        const bool running = (_got_status && (_status_word & STATUS_RUNNING) != 0);
        if (now - _home_start_ms > HOME_TIMEOUT_MS) {
            abort_home("center timeout");
        } else if (_got_status && alarmed()) {
            abort_home("alarm during center");
        } else if (step_err <= arrive && !running) {
            if (!_center_step_settling) {
                _center_step_settling = true;
                _center_step_settle_ms = now + 300;
            } else if (now >= _center_step_settle_ms) {
                _center_step_settling = false;
                if (final_err <= arrive) {
                    _state = DriveState::HOME_ZERO;
                    _got_echo = false;
                    _init_attempts = 0;
                } else {
                    _center_resend = false;
                    _state = DriveState::HOME_MOVE_CENTER;
                    _home_center_prep = 5;
                    _home_center_run_spd = true;
                }
            }
        } else if (_center_step_settling && (step_err > arrive || running)) {
            _center_step_settling = false;
        } else if (!running && step_err > arrive &&
                   (now - _last_home_retry_ms) > 2000) {
            // Resend the SAME command — do not advance the target.
            _last_home_retry_ms = now;
            _center_resend = true;
            _state = DriveState::HOME_MOVE_CENTER;
            _home_center_prep = 5;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: center resend %d (enc %d)",
                          (int)_center_step_target, (int)enc_from_origin);
        } else if ((now - _last_home_progress_ms) > 3000) {
            _last_home_progress_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: centering enc %d / %d",
                          (int)enc_from_origin, (int)_center_move_target);
            // If encoder never leaves the limit, finish with a command offset so
            // stick-center maps to mid-travel without a physical center move.
            if ((enc_from_origin > -500 && enc_from_origin < 500) &&
                (now - _home_start_ms) > 12000) {
                _steer_cmd_offset = _center_move_target;
                _speed_follow = true;
                _follow_prep = 0;
                _follow_moving = false;
                _follow_sign = 0;
                _follow_slot = 0;
                _follow_alarm_step = 0;
                _follow_restore = 0;
                _follow_mid_retried = false;
                _follow_halted = false;
                _follow_alarm_count = 0;
                _follow_peak_toward = 0;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: center fail, spd-follow %d",
                              (int)_steer_cmd_offset);
                finish_home();
            }
        }
    }

    if ((now - _last_send_ms) < SEND_INTERVAL_MS) {
        return;
    }
    if (_uart->txspace() < 22) {
        return;
    }
    _last_send_ms = now;

    // Half-duplex RS485: never send more than one Modbus frame per slot.
    if (flush_queued_motion()) {
        return;
    }

    if (_state < DriveState::RUN_WRITE) {
        if (_got_echo) {
            advance_init();
        } else {
            if (_init_attempts == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: init step %d", (int)_state);
                _last_echo_wait_ms = now;
            } else if ((now - _last_echo_wait_ms) >= WAIT_ECHO_WARN_MS) {
                _last_echo_wait_ms = now;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: init step %d waiting for echo",
                              (int)_state);
            }
            _init_attempts++;
        }
    } else if (in_run()) {
        if (!_speed_follow && _last_rx_ms != 0 && (now - _last_rx_ms) > 2000) {
            _last_rx_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: Modbus Timeout, continuing");
        }
        if (!_speed_follow && _got_status && alarmed() &&
            (now - _last_alarm_warn_ms) > 5000) {
            _last_alarm_warn_ms = now;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: alarm latched, HOME_TRIG=2");
        }
    } else if (home_prep_wait_echo()) {
        if (_got_echo) {
            advance_home();
        } else {
            if (_init_attempts == 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home prep step %d", (int)_state);
                _last_echo_wait_ms = now;
            } else if ((now - _last_echo_wait_ms) >= WAIT_ECHO_WARN_MS) {
                _last_echo_wait_ms = now;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: home step %d waiting for echo",
                              (int)_state);
            }
            _init_attempts++;
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
    case DriveState::INIT_TRACK_ERR:
        send_u16(REG_TRACK_ERR, track_err_limit());
        break;
    case DriveState::RUN_WRITE: {
        if (_alarm_clear_pending) {
            send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
            _alarm_clear_pending = false;
            _state = DriveState::RUN_READ;
            break;
        }
        if (_enable_after_alarm_clear) {
            send_u16(REG_MOTOR_ENABLE, 0x0001);
            _enable_after_alarm_clear = false;
            _state = DriveState::RUN_READ;
            break;
        }
        if (_pending_run_spd) {
            send_u16(REG_MAX_SPD, run_speed_rpm());
            _pending_run_spd = false;
            _state = DriveState::RUN_READ;
            break;
        }
        if (_pending_track_restore) {
            send_u16(REG_TRACK_ERR, track_err_limit());
            _pending_track_restore = false;
            _state = DriveState::RUN_READ;
            break;
        }
        if (_jog_active) {
            const uint32_t jnow = AP_HAL::millis();
            const int32_t moved = _actual_pulses - _jog_start_enc;
            const int32_t abs_moved = (moved >= 0) ? moved : -moved;
            if (abs_moved > 50) {
                _jog_last_progress_ms = jnow;
            }
            // Prep: STOP → CLEAR → ENABLE → SPD → SPEED, then hold/rearm.
            if (_jog_slot < 5) {
                if (_jog_slot == 0) {
                    send_u16(REG_MOTION, MOTION_STOP);
                } else if (_jog_slot == 1) {
                    send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
                } else if (_jog_slot == 2) {
                    send_u16(REG_MOTOR_ENABLE, 0x0001);
                } else if (_jog_slot == 3) {
                    const int16_t spd = (int16_t)((int32_t)calib_crawl_rpm() *
                                                 (int32_t)_jog_sign);
                    send_u16(REG_MAX_SPD, (uint16_t)spd);
                } else {
                    send_u16(REG_MOTION, MOTION_SPEED);
                    _jog_start_enc = _actual_pulses;
                    _jog_last_progress_ms = jnow;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: jog run");
                }
                _jog_slot++;
                _state = DriveState::RUN_READ;
                break;
            }
            if (abs_moved >= _jog_need) {
                send_u16(REG_MOTION, MOTION_STOP);
                _jog_active = false;
                _jog_sign = 0;
                _center_encoder_origin = _actual_pulses;
                _last_target = 0;
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: jog done moved %d", (int)moved);
                _state = DriveState::RUN_READ;
                break;
            }
            // No progress (still jammed / alarm) — clear and re-kick speed.
            if (_got_status && alarmed()) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: jog alarm, retry");
                _jog_slot = 0;
                send_u16(REG_MOTION, MOTION_STOP);
                _state = DriveState::RUN_READ;
                break;
            }
            if ((jnow - _jog_last_progress_ms) > 4000) {
                send_u16(REG_MOTION, MOTION_STOP);
                _jog_active = false;
                _jog_sign = 0;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: jog timeout moved %d", (int)moved);
                _state = DriveState::RUN_READ;
                break;
            }
            if ((jnow - _jog_start_ms) > 20000) {
                send_u16(REG_MOTION, MOTION_STOP);
                _jog_active = false;
                _jog_sign = 0;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: jog abort");
                _state = DriveState::RUN_READ;
                break;
            }
            // Keep enable alive; re-issue SPEED periodically so a cleared
            // alarm does not leave the drive idle.
            if (_jog_slot == 5) {
                send_u16(REG_MOTOR_ENABLE, 0x0001);
                _jog_slot = 6;
            } else {
                send_u16(REG_MOTION, MOTION_SPEED);
                _jog_slot = 5;
            }
            _state = DriveState::RUN_READ;
            break;
        }
        if (_speed_follow) {
            // Continuous speed-mode position follow. Absolute Bit0 starts are
            // ignored after native home on this CL57R; Bit3 speed mode moves.
            const int32_t target = stick_pulses + _steer_cmd_offset;
            const int32_t enc = _actual_pulses - _center_encoder_origin;
            const int32_t err = target - enc;
            const int32_t abs_err = (err >= 0) ? err : -err;
            const int32_t arrive_db = MAX(pos_db.get(), 800);
            // Brake window ~125ms of cruise travel, capped so high HOME_SPD
            // still reaches near mid (not stop 50k early).
            const uint16_t mid_rpm = mid_seek_speed_rpm();
            int32_t mid_stop = (int32_t)mid_rpm * CL57R_STEPS_PER_REV / 60 / 8;
            if (mid_stop < arrive_db) {
                mid_stop = arrive_db;
            }
            if (mid_stop > 12000) {
                mid_stop = 12000;
            }
            _last_target = target;

            // Track peak progress toward mid (used to avoid false "no progress").
            if (_steer_cmd_offset != 0) {
                const int32_t toward = enc * ((_steer_cmd_offset >= 0) ? 1 : -1);
                if (toward > _follow_peak_toward) {
                    _follow_peak_toward = toward;
                }
            }

            // Hard safety: never drive further than ~expected/2 from the
            // post-cal origin. Oversize mid offset used to slam the far limit.
            if (_steer_cmd_offset != 0 && _follow_prep >= 9 && _follow_alarm_step == 0) {
                const int32_t exp_full = expected_full_travel_pulses();
                if (exp_full >= 20000) {
                    const int32_t abs_enc = (enc >= 0) ? enc : -enc;
                    if (abs_enc > (exp_full / 2) + mid_stop) {
                        send_u16(REG_MOTION, MOTION_STOP);
                        _center_encoder_origin = _actual_pulses;
                        _steer_cmd_offset = 0;
                        _have_target = true;
                        _last_target = 0;
                        _follow_moving = false;
                        _follow_sign = 0;
                        _follow_slot = 0;
                        _follow_restore = 1;
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                      "CL57R: mid overshoot stop %d", (int)enc);
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: at mid-travel, ready");
                        _state = DriveState::RUN_READ;
                        break;
                    }
                }
            }

            if (_follow_prep < 9) {
                // Exit native home mode before commanding speed mode.
                if (_follow_prep == 0) {
                    send_u16(REG_MOTION, MOTION_STOP);
                } else if (_follow_prep == 1) {
                    send_u16(REG_MOTION, 0x0000);
                } else if (_follow_prep == 2) {
                    send_u16(REG_MOTOR_ENABLE, 0x0001);
                } else if (_follow_prep == 3) {
                    send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
                } else if (_follow_prep == 4) {
                    send_u16(0x0030, (uint16_t)start_speed.get());
                } else if (_follow_prep == 5) {
                    send_u16(0x0031, MID_SEEK_ACCEL_MS);
                } else if (_follow_prep == 6) {
                    send_u16(0x0032, MID_SEEK_DECEL_MS);
                } else if (_follow_prep == 7) {
                    // Do not rewrite TRACK_ERR here — 0xFFFF can raise Modbus
                    // exception code 3 (illegal data) on this CL57R.
                    send_u16(REG_MOTOR_ENABLE, 0x0001);
                } else {
                    // Do not AUX_POS_ZERO on the pressed limit — it aggravates faults.
                    _center_encoder_origin = _actual_pulses;
                    _follow_last_enc = 0;
                    _follow_last_spd = 0;
                    _follow_progress_ms = now;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                  "CL57R: spd-follow on, ofs %d",
                                  (int)_steer_cmd_offset);
                }
                _follow_prep++;
                _state = DriveState::RUN_READ;
                break;
            }

            if (_follow_restore > 0) {
                if (_follow_restore == 1) {
                    send_u16(0x0031, ACCEL_DEFAULT);
                } else if (_follow_restore == 2) {
                    send_u16(0x0032, DECEL_DEFAULT);
                } else {
                    send_u16(REG_MAX_SPD, run_speed_rpm());
                    _pending_run_spd = false;
                    _follow_restore = 0;
                    _state = DriveState::RUN_READ;
                    break;
                }
                _follow_restore++;
                _state = DriveState::RUN_READ;
                break;
            }

            if (_follow_halted) {
                // Do not keep slamming a limit after wrong-way / repeat alarms.
                // Stick deflection releases control from the current pose.
                if (stick_pulses > arrive_db || stick_pulses < -arrive_db) {
                    _follow_halted = false;
                    _follow_alarm_count = 0;
                    _steer_cmd_offset = 0;
                    _center_encoder_origin = _actual_pulses;
                    _follow_moving = false;
                    _follow_sign = 0;
                    _follow_slot = 0;
                    _pending_run_spd = true;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: follow stick takeover");
                } else {
                    send_u16(REG_MOTOR_ENABLE, 0x0001);
                }
                _state = DriveState::RUN_READ;
                break;
            }

            if (_follow_alarm_step > 0) {
                if (_follow_alarm_step == 1) {
                    send_u16(REG_MOTION, MOTION_STOP);
                    _follow_alarm_step = 2;
                } else if (_follow_alarm_step == 2) {
                    send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
                    _follow_alarm_step = 3;
                } else if (_follow_alarm_step == 3) {
                    send_u16(REG_MOTOR_ENABLE, 0x0001);
                    _follow_alarm_step = 4;
                } else if (_follow_alarm_step == 4) {
                    // If we already reached / passed mid, finish here. Never
                    // re-command a full mid distance after overshoot (v5 bug:
                    // encoder reset → remain≈-194k drove into the far stop).
                    const bool mid_return = (_steer_cmd_offset != 0);
                    const int32_t abs_goal = (_steer_cmd_offset >= 0) ?
                                            _steer_cmd_offset : -_steer_cmd_offset;
                    const bool near_or_past =
                        mid_return &&
                        (_follow_peak_toward > (abs_goal - mid_stop) ||
                         abs_err <= mid_stop ||
                         (_follow_sign < 0 && enc <= target) ||
                         (_follow_sign > 0 && enc >= target) ||
                         (_steer_cmd_offset < 0 && enc <= _steer_cmd_offset) ||
                         (_steer_cmd_offset > 0 && enc >= _steer_cmd_offset));
                    if (mid_return && near_or_past) {
                        send_u16(REG_MOTION, MOTION_STOP);
                        _center_encoder_origin = _actual_pulses;
                        _steer_cmd_offset = 0;
                        _have_target = true;
                        _last_target = 0;
                        _follow_moving = false;
                        _follow_sign = 0;
                        _follow_slot = 0;
                        _follow_alarm_step = 0;
                        _follow_alarm_count = 0;
                        _follow_halted = false;
                        _follow_restore = 1;
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: at mid-travel, ready");
                    } else if (mid_return && !_follow_mid_retried) {
                        const int32_t remain = target - enc;
                        // Refuse a near-full-travel rebase after encoder wipe.
                        const int32_t abs_remain = (remain >= 0) ? remain : -remain;
                        if (abs_remain > abs_goal / 2 && _follow_peak_toward > abs_goal / 3) {
                            _center_encoder_origin = _actual_pulses;
                            _steer_cmd_offset = 0;
                            _follow_moving = false;
                            _follow_sign = 0;
                            _follow_slot = 0;
                            _follow_alarm_step = 0;
                            _follow_halted = false;
                            _follow_restore = 1;
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                          "CL57R: mid settle (enc jump)");
                        } else {
                            _center_encoder_origin = _actual_pulses;
                            _steer_cmd_offset = remain;
                            _center_move_target = remain;
                            _follow_mid_retried = true;
                            _follow_moving = false;
                            _follow_sign = 0;
                            _follow_slot = 0;
                            _follow_last_spd = 0;
                            _follow_alarm_step = 0;
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                          "CL57R: mid rebase remain %d",
                                          (int)remain);
                        }
                    } else if (mid_return) {
                        _follow_halted = true;
                        _follow_moving = false;
                        _follow_sign = 0;
                        _follow_slot = 0;
                        _follow_alarm_step = 0;
                        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                                      "CL57R: mid halt after alarm");
                    } else if (_follow_alarm_count >= 3) {
                        _follow_halted = true;
                        _follow_moving = false;
                        _follow_sign = 0;
                        _follow_slot = 0;
                        _follow_alarm_step = 0;
                        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                                      "CL57R: follow halt after alarms");
                    } else {
                        _follow_moving = false;
                        _follow_sign = 0;
                        _follow_slot = 0;
                        _follow_alarm_step = 0;
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                      "CL57R: follow resume %d->%d",
                                      (int)enc, (int)target);
                    }
                }
                _state = DriveState::RUN_READ;
                break;
            }

            if (_got_status && alarmed()) {
                _follow_alarm_count++;
                _follow_moving = false;
                _follow_sign = 0;
                _follow_slot = 0;
                _follow_alarm_step = 1;
                send_u16(REG_MOTION, MOTION_STOP);
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: follow alarm %d->%d",
                              (int)enc, (int)target);
                _state = DriveState::RUN_READ;
                break;
            }

            // Arrived or crossed mid target — stop before the far limit.
            const bool crossed_mid = _follow_moving && _steer_cmd_offset != 0 &&
                ((_follow_sign < 0 && enc <= target) ||
                 (_follow_sign > 0 && enc >= target));
            const int32_t stop_db = (_steer_cmd_offset != 0) ? mid_stop : arrive_db;
            if (_steer_cmd_offset != 0 && (crossed_mid || abs_err <= stop_db)) {
                if (_follow_moving) {
                    send_u16(REG_MOTION, MOTION_STOP);
                    _follow_moving = false;
                    _follow_sign = 0;
                    _follow_slot = 0;
                } else if (stick_pulses <= arrive_db && stick_pulses >= -arrive_db) {
                    // Physical mid: re-base. Do NOT AUX_POS_ZERO (drive may also
                    // spontaneously re-zero — stick-center must HOLD, not chase 0).
                    _center_encoder_origin = _actual_pulses;
                    _steer_cmd_offset = 0;
                    _have_target = true;
                    _last_target = 0;
                    _follow_alarm_count = 0;
                    _follow_halted = false;
                    _follow_restore = 1;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: at mid-travel, ready");
                } else {
                    send_u16(REG_MOTOR_ENABLE, 0x0001);
                }
                _state = DriveState::RUN_READ;
                break;
            }

            // After mid: stick-center means HOLD pose. Never chase encoder zero —
            // a drive re-zero after ready made v6 drive back into the limit at
            // run RPM (enc≈+193k, target 0) and crunch the gearbox.
            if (_steer_cmd_offset == 0 &&
                stick_pulses <= arrive_db && stick_pulses >= -arrive_db) {
                if (_follow_moving) {
                    send_u16(REG_MOTION, MOTION_STOP);
                    _follow_moving = false;
                    _follow_sign = 0;
                    _follow_slot = 0;
                    _follow_last_spd = 0;
                } else if (abs_err > arrive_db) {
                    _center_encoder_origin = _actual_pulses;
                    _last_target = 0;
                    send_u16(REG_MOTOR_ENABLE, 0x0001);
                } else {
                    send_u16(REG_MOTOR_ENABLE, 0x0001);
                }
                _state = DriveState::RUN_READ;
                break;
            }

            // Stick deflected (or still seeking mid): speed-mode follow.
            {
                // One-shot notice so the GCS can confirm stick input reached the driver.
                if (_steer_cmd_offset == 0 && !_follow_moving) {
                    if (stick_pulses != _last_stick_log &&
                        (stick_pulses > arrive_db || stick_pulses < -arrive_db)) {
                        _last_stick_log = stick_pulses;
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                      "CL57R: stick cmd %d",
                                      (int)stick_pulses);
                    }
                }
                const int8_t want_sign = (err > 0) ? 1 : -1;
                if (dir_blocked(want_sign)) {
                    if (_follow_moving) {
                        send_u16(REG_MOTION, MOTION_STOP);
                        _follow_moving = false;
                        _follow_sign = 0;
                        _follow_slot = 0;
                        _follow_last_spd = 0;
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                      "CL57R: follow blocked by %s",
                                      want_sign > 0 ? "P-OT" : "N-OT");
                    } else {
                        send_u16(REG_MOTOR_ENABLE, 0x0001);
                    }
                    // If mid-return is blocked into an active limit, drop offset
                    // and hold — do not keep charging the stop.
                    if (_steer_cmd_offset != 0) {
                        _center_encoder_origin = _actual_pulses;
                        _steer_cmd_offset = 0;
                        _have_target = true;
                        _last_target = 0;
                        _follow_restore = 1;
                        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                      "CL57R: mid aborted, DI limit");
                    }
                    _state = DriveState::RUN_READ;
                    break;
                }
                // Mid return: full HOME_SPD; after an alarm rebase use crawl RPM.
                uint16_t max_rpm = (_steer_cmd_offset != 0) ?
                                   mid_seek_speed_rpm() : run_speed_rpm();
                if (_steer_cmd_offset != 0 && _follow_mid_retried) {
                    max_rpm = calib_crawl_rpm();
                }
                // Soft approach only in the last ~0.25s of travel (not a full
                // second — that made mid crawl for most of the half-travel).
                const int32_t slow_zone = MAX((int32_t)2000,
                                             (int32_t)max_rpm * CL57R_STEPS_PER_REV / 60 / 4);
                uint16_t rpm = max_rpm;
                if (slow_zone > 0 && abs_err < slow_zone) {
                    const uint16_t min_rpm = (_steer_cmd_offset != 0) ? 80 : 80;
                    rpm = (uint16_t)MAX((int32_t)min_rpm,
                                        (int32_t)max_rpm * abs_err / slow_zone);
                }
                const int16_t signed_spd = (int16_t)((int32_t)rpm * (int32_t)want_sign);
                if (_follow_moving && _follow_sign != want_sign) {
                    // Soft stop before reversing — avoids harsh direction flip.
                    send_u16(REG_MOTION, MOTION_STOP);
                    _follow_moving = false;
                    _follow_sign = 0;
                    _follow_slot = 0;
                    _follow_last_spd = 0;
                } else if (!_follow_moving || _follow_sign != want_sign) {
                    if (_follow_slot == 0) {
                        send_u16(REG_MAX_SPD, (uint16_t)signed_spd);
                        _follow_sign = want_sign;
                        _follow_last_spd = signed_spd;
                        _follow_slot = 1;
                    } else {
                        send_u16(REG_MOTION, MOTION_SPEED);
                        _follow_moving = true;
                        _follow_slot = 0;
                        _follow_progress_ms = now;
                        _follow_last_enc = enc;
                        GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                      "CL57R: follow %drpm %d/%d",
                                      (int)signed_spd,
                                      (int)enc, (int)target);
                    }
                } else if ((now - _follow_progress_ms) > 2500) {
                    _follow_progress_ms = now;
                    const int32_t moved = (enc > _follow_last_enc) ?
                                          (enc - _follow_last_enc) :
                                          (_follow_last_enc - enc);
                    _follow_last_enc = enc;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                  "CL57R: follow enc %d/%d",
                                  (int)enc, (int)target);
                    if (moved < 200) {
                        // Drive dropped speed mode — reassert without reversing.
                        send_u16(REG_MOTION, MOTION_SPEED);
                    } else {
                        send_u16(REG_MOTOR_ENABLE, 0x0001);
                    }
                } else {
                    // Refresh approach MAX_SPD only when rpm changes a lot.
                    // Spamming MAX_SPD every 50ms starves encoder RX and the
                    // firmware thinks enc stuck at 0 while the motor runs away.
                    const int16_t spd_delta = (signed_spd > _follow_last_spd) ?
                                             (signed_spd - _follow_last_spd) :
                                             (_follow_last_spd - signed_spd);
                    if (spd_delta >= 30) {
                        send_u16(REG_MAX_SPD, (uint16_t)signed_spd);
                        _follow_last_spd = signed_spd;
                    } else {
                        send_u16(REG_MOTOR_ENABLE, 0x0001);
                    }
                }
            }
            _state = DriveState::RUN_READ;
            break;
        }
        const int32_t target = stick_pulses + _steer_cmd_offset;
        const int32_t deadband = pos_db.get();
        const int32_t delta = (target > _last_target) ? (target - _last_target) : (_last_target - target);
        const bool send_pos = !_have_target || (delta > deadband);
        if (send_pos) {
            // Absolute stick target; Bit2 interrupts in-progress moves.
            send_target_pos(target);
            queue_motion(MOTION_START_ABS_IRQ);
        } else {
            send_u16(REG_MOTOR_ENABLE, 0x0001);
        }
        _state = DriveState::RUN_READ;
        break;
    }
    case DriveState::RUN_READ:
        // Rotate encoder / status / DI so limit inputs stay fresh.
        if (_read_phase == 0) {
            _rx_expect = RxExpect::ENCODER;
            modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
        } else if (_read_phase == 1) {
            _rx_expect = RxExpect::STATUS;
            modbus_create_read_packet((uint8_t)slave_id.get(), REG_STATUS, 2, tx_packet);
        } else {
            _rx_expect = RxExpect::DI;
            modbus_create_read_packet((uint8_t)slave_id.get(), REG_DI_STATE, 2, tx_packet);
        }
        _read_phase = (uint8_t)((_read_phase + 1) % 3);
        _uart->write(tx_packet, 8);
        _state = DriveState::RUN_WRITE;
        break;
    case DriveState::HOME_CLEAR_ALARM:
        send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
        break;
    case DriveState::HOME_SET_METHOD:
        send_u16(REG_HOME_METHOD, home_method_reg());
        break;
    case DriveState::HOME_SET_SPD: {
        const uint16_t spd = calib_speed_rpm();
        send_u16(REG_HOME_SPD, spd);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: write HOME_SPD=%u", (unsigned)spd);
        break;
    }
    case DriveState::HOME_SET_RUN_SPD: {
        const uint16_t spd = calib_speed_rpm();
        send_u16(REG_MAX_SPD, spd);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: write MAX_SPD=%u (calib)", (unsigned)spd);
        break;
    }
    case DriveState::HOME_SET_CRAWL: {
        const uint16_t crawl = calib_crawl_rpm();
        send_u16(REG_HOME_CRAWL, crawl);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: write HOME_CRAWL=%u", (unsigned)crawl);
        break;
    }
    case DriveState::HOME_SET_ACCEL:
        // Soft accel for speed-mode seek reduces tracking alarms under load.
        if (dual_limit_home()) {
            send_u16(0x0031, MID_SEEK_ACCEL_MS);
        } else {
            send_u16(REG_HOME_ACCEL, ACCEL_DEFAULT);
        }
        break;
    case DriveState::HOME_ENABLE:
        send_u16(REG_MOTOR_ENABLE, 0x0001);
        break;
    case DriveState::HOME_SET_TRACK_ERR: {
        const uint16_t te = seek_track_err_limit();
        send_u16(REG_TRACK_ERR, te);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: seek TRACK_ERR=%u (param %u)",
                      (unsigned)te, (unsigned)track_err_limit());
        break;
    }
    case DriveState::HOME_SEEK_SPD: {
        // Soft decel already set; signed cruise RPM for this leg.
        int16_t spd = (int16_t)calib_speed_rpm();
        if (home_method_reg() == 18) {
            spd = (int16_t)(-spd);
        }
        send_u16(REG_MAX_SPD, (uint16_t)spd);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: write seek spd=%d", (int)spd);
        break;
    }
    case DriveState::HOME_START:
        if (_home_speed_leg) {
            send_u16(REG_MOTION, MOTION_SPEED);
        } else {
            send_u16(REG_MOTION, MOTION_HOME);
        }
        break;
    case DriveState::HOME_WAIT:
        if (_home_stop_pending) {
            _home_stop_pending = false;
            send_u16(REG_MOTION, MOTION_STOP);
            _home_clear_pending = true;
            break;
        }
        if (_home_clear_pending) {
            _home_clear_pending = false;
            send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
            _home_ignore_alarm_ms = AP_HAL::millis() + 400;
            break;
        }
        if (_home_crawl_pending) {
            // After early tracking alarm: crawl same direction and resume.
            // If limit was already active at start: crawl the opposite way.
            _home_crawl_pending = false;
            int16_t spd = (int16_t)calib_crawl_rpm();
            bool neg = (home_method_reg() == 18);
            if (_home_crawl_away) {
                neg = !neg;
                _home_crawl_away = false;
            }
            if (neg) {
                spd = (int16_t)(-spd);
            }
            // Keep leg origin / peak — crawl continues the same seek span.
            // Ignore alarms briefly so STOP/CLEAR/SPEED can take effect before
            // the same latched status burns another early-retry slot.
            send_u16(REG_MAX_SPD, (uint16_t)spd);
            _queued_motion = MOTION_SPEED;
            _home_ignore_alarm_ms = AP_HAL::millis() + 1200;
            _last_home_progress_ms = AP_HAL::millis();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: seek crawl spd=%d", (int)spd);
            break;
        }
        if (_home_retry_pending) {
            // Dedicated slot: never piggy-back home restart on a status read.
            _home_retry_pending = false;
            if (_home_speed_leg) {
                int16_t spd = (int16_t)calib_speed_rpm();
                if (home_method_reg() == 18) {
                    spd = (int16_t)(-spd);
                }
                send_u16(REG_MAX_SPD, (uint16_t)spd);
                _queued_motion = MOTION_SPEED;
            } else {
                send_u16(REG_MOTION, MOTION_HOME);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: home retry");
            break;
        }
        if (_home_read_encoder) {
            // Cycle encoder / status / DI during home wait.
            if (_read_phase == 0) {
                _rx_expect = RxExpect::ENCODER;
                modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
            } else if (_read_phase == 1) {
                _rx_expect = RxExpect::STATUS;
                modbus_create_read_packet((uint8_t)slave_id.get(), REG_STATUS, 2, tx_packet);
            } else {
                _rx_expect = RxExpect::DI;
                modbus_create_read_packet((uint8_t)slave_id.get(), REG_DI_STATE, 2, tx_packet);
            }
            _read_phase = (uint8_t)((_read_phase + 1) % 3);
        } else {
            _rx_expect = RxExpect::STATUS;
            modbus_create_read_packet((uint8_t)slave_id.get(), REG_STATUS, 2, tx_packet);
        }
        _home_read_encoder = !_home_read_encoder;
        _uart->write(tx_packet, 8);
        if (!_saw_home_motion && (now - _home_start_ms) > 3000 &&
            (now - _last_home_retry_ms) > 3000) {
            _last_home_retry_ms = now;
            _home_retry_pending = true;
        }
        break;
    case DriveState::HOME_MOVE_CENTER:
        // Re-arm absolute positioning after native home before stepping.
        if (_home_center_prep == 0) {
            send_u16(REG_MOTOR_ENABLE, 0x0001);
            _home_center_prep = 1;
            break;
        }
        if (_home_center_prep == 1) {
            send_u16(REG_POS_MODE, 0x0001);
            _home_center_prep = 2;
            break;
        }
        if (_home_center_prep == 2) {
            send_u16(REG_TRACK_ERR, track_err_limit());
            _home_center_prep = 3;
            break;
        }
        if (_home_center_prep == 3) {
            send_u16(REG_MAX_SPD, calib_speed_rpm());
            _home_center_prep = 4;
            _home_center_run_spd = true;
            break;
        }
        if (_home_center_prep == 4) {
            // Sync command origin at the current limit before abs moves.
            send_u16(REG_AUX_CONTROL, AUX_POS_ZERO);
            _home_center_prep = 5;
            break;
        }
        {
            const int32_t enc_from_origin = _actual_pulses - _center_encoder_origin;
            const int32_t err = _center_move_target - enc_from_origin;
            const int32_t abs_err = (err >= 0) ? err : -err;
            int32_t arrive = pos_db.get();
            if (arrive < 500) {
                arrive = 500;
            }
            if (abs_err <= arrive) {
                _center_step_target = _center_move_target;
                _center_step_settling = false;
                _center_resend = false;
                _state = DriveState::HOME_WAIT_CENTER;
                break;
            }

            int32_t cmd;
            if (_center_resend && _center_step_target != 0) {
                cmd = _center_step_target;
            } else {
                // One absolute command for the remaining distance.
                cmd = _center_move_target;
            }
            _center_resend = false;

            send_target_pos(cmd);
            queue_motion(MOTION_START_ABS);
            _center_step_target = cmd;
            _center_step_settling = false;
            _got_status = false;
            _last_home_retry_ms = now;
            _state = DriveState::HOME_WAIT_CENTER;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: center cmd %d mot=0x%02x enc %d",
                          (int)cmd, (unsigned)MOTION_START_ABS, (int)enc_from_origin);
        }
        break;
    case DriveState::HOME_WAIT_CENTER:
        if (_home_read_encoder) {
            _rx_expect = RxExpect::ENCODER;
            modbus_create_read_packet((uint8_t)slave_id.get(), REG_ENCODER_POS, 2, tx_packet);
        } else {
            _rx_expect = RxExpect::STATUS;
            modbus_create_read_packet((uint8_t)slave_id.get(), REG_STATUS, 2, tx_packet);
        }
        _home_read_encoder = !_home_read_encoder;
        _uart->write(tx_packet, 8);
        break;
    case DriveState::HOME_ZERO_AT_L1:
        send_u16(REG_AUX_CONTROL, AUX_POS_ZERO);
        break;
    case DriveState::HOME_ZERO:
        send_u16(REG_AUX_CONTROL, AUX_POS_ZERO);
        break;
    }
}
