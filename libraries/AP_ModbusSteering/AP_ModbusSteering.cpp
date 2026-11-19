#include "AP_ModbusSteering.h"
#include "modbus_protocol.h"

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL &hal;

namespace {
constexpr const char *CL57R_FW_TAG = "v14.4-run";
constexpr uint16_t REG_STATUS = 0x0003;
constexpr uint16_t REG_DI_STATUS = 0x0005;
constexpr uint16_t REG_ENCODER_POS = 0x0007;
constexpr uint16_t REG_SUBDIVISION = 0x0023;
constexpr uint16_t REG_START_SPD = 0x0030;
constexpr uint16_t REG_ACCEL = 0x0031;
constexpr uint16_t REG_DECEL = 0x0032;
constexpr uint16_t REG_MAX_SPD = 0x0033;
constexpr uint16_t REG_TARGET_POS = 0x0034;
constexpr uint16_t REG_MOTION = 0x0036;
constexpr uint16_t REG_AUX_CONTROL = 0x0037;
constexpr uint16_t REG_MOTOR_ENABLE = 0x0038;
constexpr uint16_t REG_POS_MODE = 0x003A;
constexpr uint16_t REG_HOME_METHOD = 0x0040;
constexpr uint16_t REG_HOME_SPD = 0x0041;
constexpr uint16_t REG_HOME_CRAWL = 0x0042;
constexpr uint16_t REG_HOME_ACCEL = 0x0043;
constexpr uint16_t REG_TRACK_ERR = 0x0052;
constexpr uint16_t AUX_ALARM_CLEAR = 0x0004;
constexpr uint16_t AUX_POS_ZERO = 0x0008;
constexpr uint16_t MOTION_SPEED = 0x0008;
constexpr uint16_t MOTION_HOME = 0x0010;
constexpr uint16_t MOTION_STOP = 0x0020;
constexpr uint16_t MOTION_ESTOP = 0x0040;
constexpr uint16_t STATUS_HOME_DONE = (1U << 1);
constexpr uint16_t STATUS_RUNNING = (1U << 2);
constexpr uint16_t STATUS_ALARM = (1U << 3);
constexpr uint16_t DI_X1 = (1U << 1);
constexpr uint16_t DI_X2 = (1U << 2);
constexpr uint16_t DI_LIMIT_MASK = (DI_X1 | DI_X2);
constexpr uint16_t SUBDIVISION_PPR = 4000;
constexpr uint16_t ACCEL_MS_DEFAULT = 1000;
constexpr int32_t TRACK_ERR_DEFAULT = 4000;
constexpr uint32_t SEND_INTERVAL_MS = 50;
constexpr uint32_t READ_TIMEOUT_MS = 150;
constexpr uint32_t ECHO_TIMEOUT_MS = 300;
constexpr uint8_t INIT_MAX_ATTEMPTS = 8;
constexpr uint32_t LINK_OK_DEFAULT_MS = 2000;
constexpr uint32_t HOME_TIMEOUT_DEFAULT_MS = 90000;
constexpr uint32_t STALL_GRACE_MS = 1000;
constexpr uint32_t STALL_WINDOW_MS = 800;
constexpr int32_t STALL_MIN_PULSES = 80;
constexpr int32_t MIN_MEASURED_TRAVEL = 1000;
constexpr uint32_t BACK_MAX_MS = 4000;
constexpr int32_t BACK_RELEASE_PULSES = 1500;
constexpr int32_t BACK_CAP_PULSES = 12000;
constexpr uint8_t BACK_MAX_TRIES = 2;
constexpr int32_t CL57R_STEPS_PER_REV = 4000;
constexpr uint32_t LINK_LOST_CONFIRM_MS = 500;
constexpr uint32_t LINK_WARN_INTERVAL_MS = 2000;
constexpr int32_t MID_JUMP_PULSES = 30000;   // ignore encoder jumps (drive zeroing)
constexpr int32_t ENCODER_ZERO_WINDOW = 5000;
constexpr uint32_t FOLLOW_REASSERT_MS = 2500;
constexpr int32_t FOLLOW_MIN_MOVE_PULSES = 200;
constexpr int16_t FOLLOW_MIN_RPM = 60;
constexpr int16_t FOLLOW_RPM_UPDATE = 30;
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

    // @Param: OUT_REV
    // @DisplayName: Rudder Lock-to-Lock Turns
    // @Description: Output-shaft turns lock-to-lock. If >0, travel sanity uses OUT_REV*RATIO*4000.
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

    // @Param: MAX_STEPS
    // @DisplayName: Maximum Steering Steps
    // @Description: Half-travel pulses. Dual-limit cal overwrites with measured half-travel.
    // @User: Standard
    AP_GROUPINFO("MAX_STEPS", 3, AP_ModbusSteering, max_steps, 200000),

    // @Param: START_SPD
    // @DisplayName: Positioning start speed
    // @Description: CL57R trapezoid start speed (register 0x0030)
    // @Units: RPM
    // @Range: 2 300
    // @User: Standard
    AP_GROUPINFO("START_SPD", 4, AP_ModbusSteering, start_speed, 15),

    // @Param: MAX_SPD
    // @DisplayName: Armed run speed
    // @Description: CL57R max speed (register 0x0033) while armed / normal stick steering
    // @Units: RPM
    // @Range: 1 3000
    // @User: Standard
    AP_GROUPINFO("MAX_SPD", 5, AP_ModbusSteering, max_speed, 1300),

    // @Param: POS_DB
    // @DisplayName: Position Deadband
    // @Description: Skip a new position write if the stick target changed by less than this (pulses).
    // @Units: pulses
    // @Range: 0 50000
    // @User: Standard
    AP_GROUPINFO("POS_DB", 6, AP_ModbusSteering, pos_db, 500),

    // @Param: RET_SLEW
    // @DisplayName: Return slew (unused)
    // @Description: Unused. Kept so existing parameter storage indices stay valid.
    // @User: Advanced
    AP_GROUPINFO("RET_SLEW", 7, AP_ModbusSteering, ret_slew, 0),

    // @Param: RST_CH
    // @DisplayName: Alarm reset RC channel
    // @Description: RC channel that clears the CL57R alarm on a rising edge (PWM above 1800). 0 disables.
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("RST_CH", 10, AP_ModbusSteering, rst_ch, 0),

    // @Param: CAL_CH
    // @DisplayName: Calibration RC channel
    // @Description: RC channel that starts dual-limit calibration on a rising edge. Ignored while armed. 0 disables.
    // @Range: 0 16
    // @User: Standard
    AP_GROUPINFO("CAL_CH", 11, AP_ModbusSteering, cal_ch, 0),

    // @Param: CAL_MODE
    // @DisplayName: Calibration mode
    // @Description: Reserved. Dual-limit native MOTION_HOME is the only path in v14+.
    // @Values: 0:Unused,1:DualLimit
    // @User: Standard
    AP_GROUPINFO("CAL_MODE", 15, AP_ModbusSteering, cal_mode, 1),

    // @Param: CAL_MTH
    // @DisplayName: First limit method
    // @Description: CL57R home method for the first leg (reg 0x0040). 17 = X2 N-OT, 18 = X1 P-OT.
    // @Values: 17:NegativeLimit,18:PositiveLimit
    // @Range: 17 18
    // @User: Standard
    AP_GROUPINFO("CAL_MTH", 12, AP_ModbusSteering, cal_mth, 17),

    // @Param: HOME_SPD
    // @DisplayName: Native home SEEK speed
    // @Description: Written to CL57R 0x0041 (MOTION_HOME SEEK RPM). Also used for mid cruise.
    // @Units: RPM
    // @Range: 5 3000
    // @User: Standard
    AP_GROUPINFO("HOME_SPD", 13, AP_ModbusSteering, home_speed, 500),

    // @Param: CRAWL_SPD
    // @DisplayName: Native home crawl speed
    // @Description: Written to CL57R 0x0042 (MOTION_HOME near-limit crawl; driver max 300).
    // @Units: RPM
    // @Range: 5 300
    // @User: Standard
    AP_GROUPINFO("CRAWL_SPD", 16, AP_ModbusSteering, crawl_speed, 200),

    // @Param: CAL_TRIG
    // @DisplayName: Calibration trigger
    // @Description: GCS trigger. 1 = start calibration. 2 = clear alarm only. Ignored while armed.
    // @Values: 0:None,1:Calibrate,2:ClearAlarm
    // @User: Standard
    AP_GROUPINFO("CAL_TRIG", 14, AP_ModbusSteering, cal_trig, 0),

    // @Param: LINK_TO
    // @DisplayName: Modbus link timeout
    // @Description: No valid Modbus reply for this many ms fails link_ok() (CRC-corrupt traffic does not count). 0 uses 2000 ms for link_ok only.
    // @Units: ms
    // @Range: 0 30000
    // @User: Standard
    AP_GROUPINFO("LINK_TO", 17, AP_ModbusSteering, link_timeout, 1500),

    // @Param: CAL_TO
    // @DisplayName: Calibration leg timeout
    // @Description: Abort calibration if a leg does not finish within this many seconds. 0 keeps 90s default.
    // @Units: s
    // @Range: 0 600
    // @User: Advanced
    AP_GROUPINFO("CAL_TO", 18, AP_ModbusSteering, cal_timeout_s, 0),

    // @Param: DI_INV
    // @DisplayName: Invert limit DI X1/X2
    // @Description: Invert CL57R DI bits for X1 (P-OT) and X2 (N-OT) after read.
    // @Values: 0:Normal,1:InvertX1X2
    // @User: Standard
    AP_GROUPINFO("DI_INV", 19, AP_ModbusSteering, di_inv, 0),

    // @Param: TRK_ERR
    // @DisplayName: Drive tracking-error limit
    // @Description: Written to CL57R 0x0052. Following error beyond this many pulses raises a drive alarm (e.g. rudder jammed on the mechanical stop). Smaller trips faster.
    // @Units: pulses
    // @Range: 100 65535
    // @User: Standard
    AP_GROUPINFO("TRK_ERR", 20, AP_ModbusSteering, track_err_limit, TRACK_ERR_DEFAULT),

    // @Param: ACC_MS
    // @DisplayName: Drive accel/decel time
    // @Description: Written to CL57R 0x0031/0x0032. Short ramps at full run speed over-current the motor on a heavy rudder and the drive faults (resetting its position register).
    // @Units: ms
    // @Range: 100 2000
    // @User: Standard
    AP_GROUPINFO("ACC_MS", 23, AP_ModbusSteering, acc_ms, ACCEL_MS_DEFAULT),

    AP_GROUPEND
};

AP_ModbusSteering::AP_ModbusSteering()
{
    AP_Param::setup_object_defaults(this, var_info);
}

bool AP_ModbusSteering::homing() const
{
    return in_cal();
}

uint32_t AP_ModbusSteering::link_timeout_ms() const
{
    const int16_t to = link_timeout.get();
    if (to <= 0) {
        return 0;
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
    if (_uart == nullptr || _last_rx_ms == 0) {
        return false;
    }
    const uint32_t to = link_timeout_ms();
    const uint32_t limit = (to > 0U) ? to : LINK_OK_DEFAULT_MS;
    const uint32_t now = AP_HAL::millis();
    // Only a valid frame proves the link; CRC-garbage bus noise does not.
    return (now - _last_rx_ms) <= limit;
}

uint16_t AP_ModbusSteering::run_speed_rpm() const
{
    return (uint16_t)constrain_int16(max_speed.get(), 1, 3000);
}

uint16_t AP_ModbusSteering::home_seek_rpm() const
{
    return (uint16_t)constrain_int16(home_speed.get(), 5, 3000);
}

uint16_t AP_ModbusSteering::home_crawl_rpm() const
{
    return (uint16_t)constrain_int16(crawl_speed.get(), 5, 300);
}

uint16_t AP_ModbusSteering::home_first_method() const
{
    const int8_t m = cal_mth.get();
    return (m == 18) ? 18 : 17;
}

uint16_t AP_ModbusSteering::home_method_now() const
{
    const uint16_t first = home_first_method();
    if (_home_leg == 0) {
        return first;
    }
    return (first == 17) ? 18 : 17;
}

uint16_t AP_ModbusSteering::home_limit_bit() const
{
    return limit_bit_for_method(home_method_now());
}

uint16_t AP_ModbusSteering::limit_bit_for_method(uint16_t method) const
{
    // Method 18 = P-OT on X1; method 17 = N-OT on X2. The wiring on some
    // installs swaps which physical end that is, so callers must derive
    // direction meaning from the legs that actually completed, not from
    // this table alone.
    return (method == 18) ? DI_X1 : DI_X2;
}

int32_t AP_ModbusSteering::expected_full_travel() const
{
    if (out_rev.get() > 0 && ratio.get() > 0) {
        return (int32_t)out_rev.get() * ratio.get() * CL57R_STEPS_PER_REV;
    }
    return 0;
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

int16_t AP_ModbusSteering::backoff_signed_rpm() const
{
    // Back away from the active limit: from X1/P-OT go negative; from X2/N-OT go positive.
    const int16_t rpm = (int16_t)home_crawl_rpm();
    if ((home_limit_bit() & DI_X1) != 0) {
        return (int16_t)(-rpm);
    }
    return rpm;
}

int16_t AP_ModbusSteering::mid_cruise_rpm() const
{
    return (int16_t)home_seek_rpm();
}

int32_t AP_ModbusSteering::mid_brake_pulses() const
{
    // Fast-cruise stage ends this far before the mid. The speed switch is
    // two echo-paced phases (~0.5-0.7 s) during which the shaft keeps
    // cruising, so the window must cover that latency plus a crawl runway.
    const int32_t v = (int32_t)home_seek_rpm() * CL57R_STEPS_PER_REV / 60;
    return v * 7 / 10 + 8000;
}

int32_t AP_ModbusSteering::mid_final_pulses() const
{
    // Halt chain latency (~0.3-0.4 s) at crawl speed, with a floor.
    const int32_t v = (int32_t)FOLLOW_MIN_RPM * CL57R_STEPS_PER_REV / 60;
    return MAX((int32_t)1200, v * 2 / 5);
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
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ModbusSteer, 0);
    if (_uart != nullptr) {
        _uart->begin(115200);
        _phase = Phase::INIT_ENABLE;
        _awaiting_echo = false;
        _got_echo = false;
        _init_attempts = 0;
        _homed = false;
        const int8_t port = serial_manager.find_portnum(AP_SerialManager::SerialProtocol_ModbusSteer, 0);
        if (port >= 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: using SERIAL%d %s", (int)port, CL57R_FW_TAG);
        }
    }
}

void AP_ModbusSteering::send_u16(uint16_t reg, uint16_t value)
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_packet[8];
    modbus_create_write_packet((uint8_t)slave_id.get(), reg, value, tx_packet);
    // UART writes can be partial when the TX buffer is full; a truncated
    // frame is never answered and stalls the whole exchange.
    write_all(tx_packet, 8);
    _last_send_ms = AP_HAL::millis();
    _awaiting_echo = true;
    _got_echo = false;
    _rx_expect = RxExpect::NONE;
}

void AP_ModbusSteering::send_read(uint16_t reg, uint16_t count, RxExpect expect)
{
    if (_uart == nullptr) {
        return;
    }
    uint8_t tx_packet[8];
    modbus_create_read_packet((uint8_t)slave_id.get(), reg, count, tx_packet);
    write_all(tx_packet, 8);
    _rx_expect = expect;
    _last_send_ms = AP_HAL::millis();
    _last_read_sent_ms = _last_send_ms;
    _awaiting_echo = false;
    _got_echo = false;
}

void AP_ModbusSteering::write_all(const uint8_t *buf, uint16_t len)
{
    uint16_t off = 0;
    uint16_t guard = 0;
    while (off < len && guard < 200) {
        const ssize_t n = _uart->write(buf + off, len - off);
        if (n > 0) {
            off += (uint16_t)n;
        } else {
            guard++;
            if (guard == 20) {
                // Diagnose slow TX: a full buffer means the SITL/HAL drain
                // stalled; otherwise write() is misbehaving.
                static uint32_t last_slow_ms;
                const uint32_t tnow = AP_HAL::millis();
                if (tnow - last_slow_ms > 1000) {
                    last_slow_ms = tnow;
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                                  "CL57R: tx slow off=%u txspace=%u",
                                  (unsigned)off, (unsigned)_uart->txspace());
                }
            }
            hal.scheduler->delay(1);
        }
    }
}

void AP_ModbusSteering::send_ff(uint16_t reg, uint16_t value)
{
    if (_uart == nullptr) {
        return;
    }
    // Fire-and-forget write: used for the run follow, where waiting for an
    // echo per speed update halves the control bandwidth and the loop goes
    // unstable at full run speed. The reply still refreshes _last_rx_ms.
    uint8_t tx_packet[8];
    modbus_create_write_packet((uint8_t)slave_id.get(), reg, value, tx_packet);
    write_all(tx_packet, 8);
    _last_send_ms = AP_HAL::millis();
}

void AP_ModbusSteering::consume_rx()
{
    if (_uart == nullptr) {
        return;
    }

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
            _rx_crc_err++;
            offset++;
            continue;
        }

        _last_rx_ms = AP_HAL::millis();

        const uint8_t fn = _rx_buf[offset + 1];
        if ((fn & 0x80) != 0) {
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
            // Slave rejected the write: the phase must not advance on it.
            if (_awaiting_echo) {
                _got_exception = true;
            }
        } else if (fn == 0x06 || fn == 0x10) {
            // Keep _awaiting_echo set until advance_after_echo() consumes it;
            // clearing here skipped the advance path and spun forever on INIT.
            // Only accept while a write is outstanding, so a late duplicate
            // echo cannot advance the phase a second time.
            if (_awaiting_echo) {
                _got_echo = true;
            }
        } else if (fn == 0x03 && frame_len >= 7) {
            const uint8_t byte_count = _rx_buf[offset + 2];
            if (_rx_expect == RxExpect::STATUS && byte_count >= 2) {
                _status_word = ((uint16_t)_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4];
                if (byte_count >= 4) {
                    // Second register is the drive error code (0x0004).
                    _err_code = (uint16_t)((_rx_buf[offset + 5] << 8) | _rx_buf[offset + 6]);
                }
                _got_status = true;
                if (_follow_stop_wait) {
                    _follow_stop_status_seen = true;
                }
            } else if (_rx_expect == RxExpect::DI_INPUT && byte_count >= 2) {
                const uint16_t raw = ((uint16_t)_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4];
                uint16_t di = raw;
                if (di_inv.get() != 0) {
                    di = (uint16_t)(di ^ DI_LIMIT_MASK);
                }
                _di_word = di;
                _got_di = true;
            } else if (_rx_expect == RxExpect::ENCODER && byte_count == 0x04 && frame_len >= 9) {
                const uint16_t high_word = (_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4];
                const uint16_t low_word = (_rx_buf[offset + 5] << 8) | _rx_buf[offset + 6];
                const int32_t raw = (int32_t)(((uint32_t)high_word << 16) | low_word);
                const int32_t max_step = MAX((int32_t)2000,
                                             abs((int32_t)_follow_last_rpm) * CL57R_STEPS_PER_REV / 120);
                // This drive may zero its position register on speed-mode STOP/START.
                // Preserve world position, but never apply this during homing.
                if (in_run() && abs(raw - _encoder_raw) > max_step &&
                    abs(raw) < ENCODER_ZERO_WINDOW) {
                    _encoder_offset += _encoder_raw;
                    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                                  "CL57R: encoder zeroed, offset %d", (int)_encoder_offset);
                }
                _encoder_raw = raw;
                _actual_pulses = raw + _encoder_offset;
            } else if (_rx_expect == RxExpect::MAX_SPD && byte_count >= 2) {
                _max_speed_feedback = (int16_t)(((uint16_t)_rx_buf[offset + 3] << 8) | _rx_buf[offset + 4]);
            }
            _rx_expect = RxExpect::NONE;
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

void AP_ModbusSteering::clear_cal_trig()
{
    if (cal_trig.get() != 0) {
        cal_trig.set_and_save(0);
    }
}

void AP_ModbusSteering::send_current()
{
    switch (_phase) {
    case Phase::INIT_ENABLE:
        send_u16(REG_MOTOR_ENABLE, 0x0001);
        break;
    case Phase::INIT_CLEAR_ALARM:
        send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
        break;
    case Phase::INIT_SUBDIVISION:
        send_u16(REG_SUBDIVISION, SUBDIVISION_PPR);
        break;
    case Phase::INIT_START_SPD:
        send_u16(REG_START_SPD, (uint16_t)constrain_int16(start_speed.get(), 2, 300));
        break;
    case Phase::INIT_MAX_SPD:
        send_u16(REG_MAX_SPD, run_speed_rpm());
        break;
    case Phase::INIT_ACCEL:
        send_u16(REG_ACCEL, (uint16_t)constrain_int16(acc_ms.get(), 100, 2000));
        break;
    case Phase::INIT_DECEL:
        send_u16(REG_DECEL, (uint16_t)constrain_int16(acc_ms.get(), 100, 2000));
        break;
    case Phase::INIT_ABS_MODE:
        send_u16(REG_POS_MODE, 0x0001);
        break;
    case Phase::INIT_TRACK_ERR:
        send_u16(REG_TRACK_ERR, (uint16_t)constrain_int32(track_err_limit.get(), 100, 65535));
        break;

    case Phase::CAL_STOP:
    case Phase::MID_STOP:
    case Phase::MID_HALT:
    case Phase::BACK_STOP:
        send_u16(REG_MOTION, _use_estop ? MOTION_ESTOP : MOTION_STOP);
        break;
    case Phase::ABORT_STOP:
        send_u16(REG_MOTION, _use_estop ? MOTION_ESTOP : MOTION_STOP);
        break;
    case Phase::CAL_CLEAR:
    case Phase::MID_CLEAR:
    case Phase::BACK_CLEAR:
        send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
        break;
    case Phase::CAL_ENABLE:
    case Phase::MID_ENABLE:
    case Phase::BACK_ENABLE:
        send_u16(REG_MOTOR_ENABLE, 0x0001);
        break;
    case Phase::CAL_METHOD:
        send_u16(REG_HOME_METHOD, home_method_now());
        break;
    case Phase::CAL_HOME_SPD:
        send_u16(REG_HOME_SPD, home_seek_rpm());
        break;
    case Phase::CAL_CRAWL:
        send_u16(REG_HOME_CRAWL, home_crawl_rpm());
        break;
    case Phase::CAL_HOME_ACCEL:
        send_u16(REG_HOME_ACCEL, (uint16_t)constrain_int16(acc_ms.get(), 100, 2000));
        break;
    case Phase::CAL_START_HOME:
        send_u16(REG_MOTION, MOTION_HOME);
        break;
    case Phase::MID_MAX_SPD:
        send_u16(REG_MAX_SPD, home_seek_rpm());
        break;
    case Phase::MID_SPD:
        // Fast cruise for the first stage, crawl once inside the brake window.
        send_u16(REG_MAX_SPD, (uint16_t)(_mid_dir * (_mid_slow ? FOLLOW_MIN_RPM : mid_cruise_rpm())));
        break;
    case Phase::MID_GO:
        send_u16(REG_MOTION, MOTION_SPEED);
        break;
    case Phase::BACK_SPD:
        send_u16(REG_MAX_SPD, (uint16_t)backoff_signed_rpm());
        break;
    case Phase::BACK_START:
        send_u16(REG_MOTION, MOTION_SPEED);
        break;
    case Phase::RUN_STOP:
        send_u16(REG_MOTION, MOTION_STOP);
        break;
    case Phase::IDLE_SET_SPD:
        send_u16(REG_MAX_SPD, run_speed_rpm());
        break;

    case Phase::IDLE_UNCAL:
    case Phase::IDLE_READY:
    case Phase::CAL_ALARM_WAIT:
    case Phase::CAL_WAIT:
    case Phase::MID_FOLLOW:
    case Phase::MID_ZERO:
    case Phase::BACK_POLL:
        break;
    }
}

void AP_ModbusSteering::advance_after_echo()
{
    _got_echo = false;
    _got_exception = false;
    _awaiting_echo = false;
    _init_attempts = 0;

    switch (_phase) {
    case Phase::INIT_ENABLE:
        _phase = Phase::INIT_CLEAR_ALARM;
        break;
    case Phase::INIT_CLEAR_ALARM:
        _phase = Phase::INIT_SUBDIVISION;
        break;
    case Phase::INIT_SUBDIVISION:
        _phase = Phase::INIT_START_SPD;
        break;
    case Phase::INIT_START_SPD:
        _phase = Phase::INIT_MAX_SPD;
        break;
    case Phase::INIT_MAX_SPD:
        _phase = Phase::INIT_ACCEL;
        break;
    case Phase::INIT_ACCEL:
        _phase = Phase::INIT_DECEL;
        break;
    case Phase::INIT_DECEL:
        _phase = Phase::INIT_ABS_MODE;
        break;
    case Phase::INIT_ABS_MODE:
        _phase = Phase::INIT_TRACK_ERR;
        break;
    case Phase::INIT_TRACK_ERR:
        _phase = Phase::IDLE_UNCAL;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: init OK %s", CL57R_FW_TAG);
        break;

    case Phase::CAL_STOP:
        _phase = Phase::CAL_CLEAR;
        break;
    case Phase::CAL_CLEAR:
        _phase = Phase::CAL_ENABLE;
        break;
    case Phase::CAL_ENABLE:
        _phase = Phase::CAL_METHOD;
        break;
    case Phase::CAL_METHOD:
        _phase = Phase::CAL_HOME_SPD;
        break;
    case Phase::CAL_HOME_SPD:
        _phase = Phase::CAL_CRAWL;
        break;
    case Phase::CAL_CRAWL:
        _phase = Phase::CAL_HOME_ACCEL;
        break;
    case Phase::CAL_HOME_ACCEL:
        _phase = Phase::CAL_START_HOME;
        break;
    case Phase::CAL_START_HOME:
        _phase = Phase::CAL_WAIT;
        _leg_start_ms = AP_HAL::millis();
        _last_progress_ms = _leg_start_ms;
        _last_progress_enc = _actual_pulses;
        _leg_peak_travel = 0;
        _saw_home_clear = false;
        _saw_home_run = false;
        _saw_limit = false;
        _got_status = false;
        _got_di = false;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: seek %u M%u %urpm",
                      (unsigned)(_home_leg + 1),
                      (unsigned)home_method_now(),
                      (unsigned)home_seek_rpm());
        break;

    case Phase::MID_STOP:
        _phase = Phase::MID_CLEAR;
        break;
    case Phase::MID_CLEAR:
        _phase = Phase::MID_ENABLE;
        break;
    case Phase::MID_ENABLE:
        _phase = Phase::MID_MAX_SPD;
        break;
    case Phase::MID_MAX_SPD:
        // Speed-follow mid run, opposite to the leg2 seek direction. The
        // distance is measured from the encoder because some units do not
        // zero after the second home.
        _phase = Phase::MID_SPD;
        _mid_origin_enc = _actual_pulses;
        _mid_last_enc = _actual_pulses;
        _mid_traveled = 0;
        _mid_dir = (_leg_dir >= 0) ? -1 : 1;
        _mid_slow = false;
        _mid_zero_sent = false;
        _leg_start_ms = AP_HAL::millis();
        _last_progress_ms = _leg_start_ms;
        break;
    case Phase::MID_SPD:
        _phase = Phase::MID_GO;
        break;
    case Phase::MID_GO:
        _phase = Phase::MID_FOLLOW;
        break;
    case Phase::MID_HALT:
        _phase = Phase::MID_ZERO;
        break;
    case Phase::MID_ZERO:
        // Mid zero done; restore armed run speed before declaring ready.
        _phase = Phase::IDLE_SET_SPD;
        break;

    case Phase::BACK_STOP:
        _phase = Phase::BACK_CLEAR;
        break;
    case Phase::BACK_CLEAR:
        _phase = Phase::BACK_ENABLE;
        break;
    case Phase::BACK_ENABLE:
        _phase = Phase::BACK_SPD;
        break;
    case Phase::BACK_SPD:
        _phase = Phase::BACK_START;
        break;
    case Phase::BACK_START:
        _phase = Phase::BACK_POLL;
        _back_start_ms = AP_HAL::millis();
        _last_progress_enc = _actual_pulses;
        break;

    case Phase::ABORT_STOP:
        _phase = Phase::IDLE_UNCAL;
        _homed = false;
        _use_estop = false;
        clear_cal_trig();
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: cal aborted -> idle");
        break;

    case Phase::RUN_STOP:
        _phase = Phase::IDLE_READY;
        break;
    case Phase::IDLE_SET_SPD:
        finish_cal();
        break;

    case Phase::IDLE_UNCAL:
    case Phase::IDLE_READY:
    case Phase::CAL_ALARM_WAIT:
    case Phase::CAL_WAIT:
    case Phase::MID_FOLLOW:
    case Phase::BACK_POLL:
        break;
    }
}

void AP_ModbusSteering::start_home()
{
    if (hal.util->get_soft_armed()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: cal ignored, armed");
        clear_cal_trig();
        return;
    }
    if (in_cal()) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: cal already running");
        return;
    }
    _homed = false;
    _home_leg = 0;
    _back_tries = 0;
    _use_estop = false;
    _measured_half_travel = 0;
    _leg1_travel = 0;
    _leg_dir = 0;
    _encoder_raw = 0;
    _encoder_offset = 0;
    _cal_start_ms = AP_HAL::millis();
    if (alarmed()) {
        // Recover from a latched drive alarm without needing CAL_TRIG=2
        // first; the wait phase polls the status word until the bit drops.
        _phase = Phase::CAL_ALARM_WAIT;
        _awaiting_echo = false;
        _got_echo = false;
        _init_attempts = 0;
        send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: cal start, clearing alarm");
    } else {
        _phase = Phase::CAL_STOP;
        _awaiting_echo = false;
        _got_echo = false;
        _init_attempts = 0;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: cal start M%u %s",
                  (unsigned)home_first_method(), CL57R_FW_TAG);
}

void AP_ModbusSteering::begin_leg_prep()
{
    _use_estop = false;
    _phase = Phase::CAL_STOP;
    _awaiting_echo = false;
    _got_echo = false;
    _init_attempts = 0;
}

void AP_ModbusSteering::begin_mid()
{
    // This CL57R ignores absolute positioning starts after native home, so
    // the mid run is speed-mode follow (see evaluate_mid_follow).
    _use_estop = false;
    // Record which limit input guards which encoder end, proven by the leg
    // that just completed: leg2 ended in the _leg_dir direction at the
    // switch selected by its home method. The run backstop uses this map
    // instead of assuming the wiring matches the register documentation.
    const uint16_t near_bit = limit_bit_for_method(home_method_now());
    const uint16_t other_bit = (near_bit == DI_X1) ? DI_X2 : DI_X1;
    if (_leg_dir < 0) {
        _neg_end_bit = near_bit;
        _pos_end_bit = other_bit;
    } else {
        _pos_end_bit = near_bit;
        _neg_end_bit = other_bit;
    }
    _phase = Phase::MID_STOP;
    _awaiting_echo = false;
    _got_echo = false;
    _init_attempts = 0;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: mid target %d half %d dir %d legdir %d",
                  (int)_mid_target, (int)_measured_half_travel,
                  (int)_mid_dir, (int)_leg_dir);
}

void AP_ModbusSteering::begin_backoff(const char *why)
{
    if (_back_tries >= BACK_MAX_TRIES) {
        abort_home(why, true);
        return;
    }
    _back_tries++;
    _use_estop = false;
    _phase = Phase::BACK_STOP;
    _awaiting_echo = false;
    _got_echo = false;
    _init_attempts = 0;
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: backoff %u (%s)",
                  (unsigned)_back_tries, why);
}

void AP_ModbusSteering::abort_home(const char *reason, bool estop)
{
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: %s", reason);
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                  "CL57R: abort di=%04x st=%04x enc=%d",
                  (unsigned)_di_word,
                  (unsigned)_status_word,
                  (int)_actual_pulses);
    _use_estop = estop;
    _homed = false;
    _phase = Phase::ABORT_STOP;
    _awaiting_echo = false;
    _got_echo = false;
    _init_attempts = 0;
    clear_cal_trig();
}

void AP_ModbusSteering::finish_cal()
{
    max_steps.set_and_save(_measured_half_travel);
    _homed = true;
    // The AUX POS_ZERO echo means the encoder now reads zero at mid; act on
    // it immediately instead of waiting for the next poll (a stale reading
    // here would kick the follow into a spurious correction run).
    _actual_pulses = 0;
    _encoder_raw = 0;
    _encoder_offset = 0;
    _follow_moving = false;
    _follow_speed_pending = false;
    _follow_stop_wait = false;
    _follow_sign = 0;
    _run_link_failsafe = false;
    _run_limit_hold = false;
    _phase = Phase::IDLE_READY;
    _awaiting_echo = false;
    _got_echo = false;
    _init_attempts = 0;
    clear_cal_trig();
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,
                  "CL57R: calibrated at mid half=%d %s",
                  (int)_measured_half_travel, CL57R_FW_TAG);
}

void AP_ModbusSteering::evaluate_cal_wait(uint32_t now)
{
    if ((now - _leg_start_ms) > home_timeout_ms()) {
        abort_home("cal leg timeout", true);
        return;
    }
    if ((_status_word & STATUS_ALARM) != 0) {
        begin_backoff("alarm on seek");
        return;
    }

    if ((_status_word & STATUS_HOME_DONE) == 0) {
        _saw_home_clear = true;
    }
    if ((_status_word & STATUS_RUNNING) != 0) {
        _saw_home_run = true;
    }
    if ((_di_word & home_limit_bit()) != 0) {
        _saw_limit = true;
    }

    // Native MOTION_HOME zeros the encoder at the limit; capture peak |enc|
    // during the seek so lock-to-lock travel survives that zeroing.
    const int32_t abs_enc = abs(_actual_pulses);
    if (abs_enc > _leg_peak_travel) {
        _leg_peak_travel = abs_enc;
    }

    const int32_t moved = abs(_actual_pulses - _last_progress_enc);
    if (moved >= STALL_MIN_PULSES) {
        // Direction only from plausible motion: the drive zeroes its encoder
        // at the limit, and that jump must not rewrite the seek direction.
        if (moved < MID_JUMP_PULSES) {
            _leg_dir = (_actual_pulses >= _last_progress_enc) ? 1 : -1;
        }
        _last_progress_enc = _actual_pulses;
        _last_progress_ms = now;
    } else if ((now - _leg_start_ms) > STALL_GRACE_MS &&
               (now - _last_progress_ms) > STALL_WINDOW_MS) {
        begin_backoff("seek stall");
        return;
    }

    // Done: home-done rose after clear, and the expected limit is active.
    if (_saw_home_clear &&
        (_status_word & STATUS_HOME_DONE) != 0 &&
        _saw_limit) {
        if (_home_leg == 0) {
            _leg1_travel = _leg_peak_travel;
            _home_leg = 1;
            _back_tries = 0;
            begin_leg_prep();
            return;
        }

        // Second limit: peak travel from the first zeroed limit to this one.
        const int32_t full = _leg_peak_travel;
        if (full < MIN_MEASURED_TRAVEL) {
            abort_home("travel too small", true);
            return;
        }
        const int32_t expected = expected_full_travel();
        if (expected >= 20000) {
            if (full < expected / 4 || full > expected * 5) {
                abort_home("travel out of range", true);
                return;
            }
        }
        _measured_half_travel = full / 2;
        // After leg2 method 18, mid is negative; after method 17, mid is positive.
        if (home_method_now() == 18) {
            _mid_target = -_measured_half_travel;
        } else {
            _mid_target = _measured_half_travel;
        }
        begin_mid();
    }

    (void)_saw_home_run;
    (void)_leg1_travel;
}

void AP_ModbusSteering::evaluate_mid_follow(uint32_t now)
{
    if ((now - _leg_start_ms) > home_timeout_ms()) {
        abort_home("mid timeout", true);
        return;
    }
    if ((_status_word & STATUS_ALARM) != 0) {
        abort_home("alarm on mid", true);
        return;
    }
    // Never drive into a limit input: the mid run travels away from the
    // leg2 end toward mid, so reaching the FAR end (leg1's limit, proven by
    // the leg that just completed) means the half travel was wrong. Both
    // bits active at once (stuck/floating switches) is inconclusive.
    const uint16_t di = _di_word;
    const uint16_t near_limit = limit_bit_for_method(home_method_now());
    const uint16_t far_limit = (near_limit == DI_X1) ? DI_X2 : DI_X1;
    if (((di & far_limit) != 0) && ((di & near_limit) == 0)) {
        abort_home("mid limit hit", true);
        return;
    }

    // Accumulate travel, ignoring implausible encoder jumps (the drive may
    // zero its position reference at any time).
    const int32_t d = _actual_pulses - _mid_last_enc;
    if (abs(d) < MID_JUMP_PULSES) {
        _mid_traveled += abs(d);
        if (abs(d) >= STALL_MIN_PULSES) {
            _last_progress_ms = now;
        }
    }
    _mid_last_enc = _actual_pulses;

    if (!_mid_slow && (_mid_traveled >= _measured_half_travel - mid_brake_pulses())) {
        // Inside the brake window: drop to crawl so the halt (two echo
        // cycles) cannot overshoot the mid by thousands of pulses.
        _mid_slow = true;
        _phase = Phase::MID_SPD;
        _awaiting_echo = false;
        _got_echo = false;
        _init_attempts = 0;
        return;
    }

    if (_mid_traveled >= _measured_half_travel - mid_final_pulses()) {
        // At the mid: stop, then zero the encoder wherever the shaft halts.
        _use_estop = false;
        _phase = Phase::MID_HALT;
        _awaiting_echo = false;
        _got_echo = false;
        _init_attempts = 0;
        return;
    }

    if ((now - _last_progress_ms) > FOLLOW_REASSERT_MS) {
        // No encoder progress: the drive may have dropped speed mode.
        _last_progress_ms = now;
        _phase = Phase::MID_GO;
        _awaiting_echo = false;
        _got_echo = false;
        _init_attempts = 0;
    }
}

void AP_ModbusSteering::evaluate_cal_alarm_wait(uint32_t now)
{
    if (_got_echo) {
        // The AUX clear write is fire-and-forget here: an echo only confirms
        // the register write, so just unblock the poll cycle with it.
        _got_echo = false;
        _awaiting_echo = false;
        _init_attempts = 0;
    }
    if ((_status_word & STATUS_ALARM) == 0) {
        _phase = Phase::CAL_STOP;
        _awaiting_echo = false;
        _got_echo = false;
        _init_attempts = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: alarm cleared, cal begin");
        return;
    }
    if ((now - _last_send_ms) > 1000) {
        // The clear may have been lost while the drive was latched, and a
        // drive that is still executing motion may need a stop before it
        // accepts the clear — alternate the two writes.
        if (_cal_alarm_stop_next) {
            send_u16(REG_MOTION, MOTION_STOP);
        } else {
            send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
        }
        _cal_alarm_stop_next = !_cal_alarm_stop_next;
    }
    bus_idle_poll(now);
    if ((now - _cal_start_ms) > 10000) {
        abort_home("alarm clear timeout", false);
    }
}

void AP_ModbusSteering::evaluate_mid_zero(uint32_t now)
{
    if ((_status_word & STATUS_ALARM) != 0) {
        abort_home("alarm before mid zero", true);
        return;
    }
    if (!_mid_zero_sent) {
        if ((_status_word & STATUS_RUNNING) != 0) {
            // The shaft is still decelerating from the halt. This unit
            // latches a fault if the AUX pos-zero lands while the running
            // bit is still set, so hold STOP and poll status until it drops.
            if ((now - _last_send_ms) > 100) {
                send_u16(REG_MOTION, MOTION_STOP);
            }
            bus_idle_poll(now);
            return;
        }
        _mid_zero_sent = true;
        send_u16(REG_AUX_CONTROL, AUX_POS_ZERO);
        return;
    }
    bus_idle_poll(now);
}

void AP_ModbusSteering::evaluate_back_poll(uint32_t now)
{
    const int32_t traveled = abs(_actual_pulses - _last_progress_enc);
    const bool limit_clear = (_di_word & home_limit_bit()) == 0;
    if ((limit_clear && traveled >= BACK_RELEASE_PULSES) ||
        traveled >= BACK_CAP_PULSES ||
        (now - _back_start_ms) > BACK_MAX_MS) {
        // Stop and re-enter seek prep for the same leg.
        _use_estop = false;
        _phase = Phase::CAL_STOP;
        _awaiting_echo = false;
        _got_echo = false;
        _init_attempts = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: backoff done, reseek");
        return;
    }
    if ((_status_word & STATUS_ALARM) != 0) {
        abort_home("alarm during backoff", true);
    }
}

void AP_ModbusSteering::bus_idle_poll(uint32_t now)
{
    if (_awaiting_echo) {
        return;
    }
    // One outstanding read at a time: a late answer to an older poll must
    // not be mistaken for the answer to a newer one.
    if (_rx_expect != RxExpect::NONE) {
        if ((now - _last_send_ms) < READ_TIMEOUT_MS) {
            return;
        }
        _rx_expect = RxExpect::NONE;
    }
    if (_last_send_ms != 0 && (now - _last_send_ms) < SEND_INTERVAL_MS) {
        return;
    }

    switch (_run_poll_phase) {
    case 0:
        send_read(REG_ENCODER_POS, 2, RxExpect::ENCODER);
        _run_poll_phase = 1;
        break;
    case 1:
        send_read(REG_STATUS, 2, RxExpect::STATUS);
        _run_poll_phase = 2;
        break;
    case 2:
        send_read(REG_DI_STATUS, 1, RxExpect::DI_INPUT);
        _run_poll_phase = 3;
        break;
    default:
        send_read(REG_MAX_SPD, 1, RxExpect::MAX_SPD);
        _run_poll_phase = 0;
        break;
    }
}

void AP_ModbusSteering::handle_run_link_failsafe(uint32_t now)
{
    if (!_homed) {
        return;
    }
    const uint32_t to = link_timeout_ms();
    if (to == 0U || _last_rx_ms == 0U) {
        _link_lost_candidate_ms = 0;
        return;
    }
    const uint32_t good_age = now - _last_rx_ms;
    if (good_age <= to) {
        _link_lost_candidate_ms = 0;
        if (_run_link_failsafe) {
            _run_link_failsafe = false;
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE,
                          "CL57R: Modbus link restored (crc_err=%u)",
                          (unsigned)_rx_crc_err);
        }
        return;
    }

    if (_link_lost_candidate_ms == 0U) {
        _link_lost_candidate_ms = now;
        return;
    }
    if ((now - _link_lost_candidate_ms) < LINK_LOST_CONFIRM_MS) {
        return;
    }

    if (!_run_link_failsafe) {
        _run_link_failsafe = true;
        _follow_moving = false;
        _follow_speed_pending = false;
        _follow_sign = 0;
        _phase = Phase::RUN_STOP;
        _awaiting_echo = false;
        _got_echo = false;
        _init_attempts = 0;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "CL57R: Modbus link lost, STOP (silent %ums crc_err=%u phase=%u)",
                      (unsigned)good_age, (unsigned)_rx_crc_err, (unsigned)_phase);
        _last_run_link_warn_ms = now;
        return;
    }

    if (_last_run_link_warn_ms == 0U ||
        (now - _last_run_link_warn_ms) >= LINK_WARN_INTERVAL_MS) {
        _last_run_link_warn_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,
                      "CL57R: Modbus link still down (crc_err=%u)",
                      (unsigned)_rx_crc_err);
    }
}

void AP_ModbusSteering::stop_follow_leg(uint32_t now)
{
    send_ff(REG_MOTION, MOTION_STOP);
    _follow_moving = false;
    _follow_speed_pending = false;
    _follow_sign = 0;
    _follow_fast = false;
    _follow_stop_wait = true;
    _follow_stop_status_seen = false;
    _follow_stop_timeout_reported = false;
    _follow_stop_ms = now;
}

void AP_ModbusSteering::handle_run(uint32_t now, float steering_out)
{
    handle_run_link_failsafe(now);
    if (_run_link_failsafe) {
        if (_phase == Phase::IDLE_READY) {
            bus_idle_poll(now);
        }
        return;
    }
    if (alarmed()) {
        if (_follow_moving) {
            // A faulted drive may still be executing its last speed leg;
            // drop it out of speed mode so the alarm can actually clear.
            send_ff(REG_MOTION, MOTION_STOP);
            _follow_moving = false;
            _follow_speed_pending = false;
            _follow_sign = 0;
        }
        _follow_stop_wait = false;
        bus_idle_poll(now);
        return;
    }

    if (_phase == Phase::RUN_STOP) {
        // Write sequence driven by the echo path in service(). Do not poll
        // here: a read would win the 50 ms send slot every tick and starve
        // the pending write forever.
        return;
    }

    if (_follow_stop_wait) {
        if (_follow_stop_status_seen && (_status_word & STATUS_RUNNING) == 0) {
            _follow_stop_wait = false;
            bus_idle_poll(now);
            return;
        }
        const uint32_t stop_timeout = (uint32_t)constrain_int16(acc_ms.get(), 100, 2000) + 3000U;
        if (!_follow_stop_timeout_reported && now - _follow_stop_ms > stop_timeout) {
            _follow_stop_timeout_reported = true;
            send_ff(REG_MOTION, MOTION_STOP);
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "CL57R: follow stop timeout st=%04x", (unsigned)_status_word);
        }
        bus_idle_poll(now);
        return;
    }

    const int32_t fast_v = (int32_t)run_speed_rpm() * CL57R_STEPS_PER_REV / 60;
    const int32_t decel_ms = constrain_int16(acc_ms.get(), 100, 2000);
    // Deceleration distance plus 100 ms control latency and one 200 ms DI poll.
    const int32_t limit_margin = fast_v * decel_ms / 2000 + fast_v * 3 / 10;
    const int32_t limit = MAX((int32_t)0, travel_limit_pulses() - limit_margin);
    const float stick = constrain_float(steering_out, -1.0f, 1.0f);
    const int32_t target = (int32_t)(stick * (float)limit);
    _debug_target = target;

    // Hard backstop: never command motion further into an active limit
    // input. Which input guards which end is learned from calibration
    // (_pos_end_bit/_neg_end_bit) — the wiring may not match the register
    // documentation.
    const bool into_pos_stop = (_di_word & _pos_end_bit) != 0 && target > _actual_pulses;
    const bool into_neg_stop = (_di_word & _neg_end_bit) != 0 && target < _actual_pulses;
    if (into_pos_stop || into_neg_stop) {
        if (!_run_limit_hold) {
            _run_limit_hold = true;
            _follow_moving = false;
            _follow_speed_pending = false;
            _follow_sign = 0;
            _phase = Phase::RUN_STOP;
            _awaiting_echo = false;
            _got_echo = false;
            _init_attempts = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                          "CL57R: limit DI hit in run, hold (di=%04x enc=%d)",
                          (unsigned)_di_word, (int)_actual_pulses);
        }
        // Hold state keeps polling via the IDLE_READY path once the STOP
        // write has gone out; do not poll here or the STOP itself starves.
        return;
    }
    _run_limit_hold = false;

    // Two-phase leg arming: MAX_SPD goes out as an awaited write (the run
    // loop must know the drive really accepted it — a lost MAX_SPD leaves
    // the old speed in effect, and this drive ignores changes mid-run), and
    // only after its echo does the SPEED trigger follow.
    if (_follow_speed_pending) {
        if (_got_echo) {
            // MAX_SPD confirmed by the drive.
            _got_echo = false;
            _awaiting_echo = false;
            _init_attempts = 0;
        }
        if (!_awaiting_echo) {
            // Either the echo just landed or the service layer already
            // consumed it; either way the register is set, so fire SPEED.
            _follow_speed_pending = false;
            send_ff(REG_MOTION, MOTION_SPEED);
            _follow_moving = true;
        }
        return;
    }

    // Speed-mode follow: this CL57R ignores absolute positioning starts and
    // also pays no attention to MAX_SPD changes while speed mode is running,
    // so the move is chunked: a fast leg that ends with an explicit STOP
    // while still far enough to coast in, then a creep leg that stops in the
    // arrive window. Every speed takes effect because it is written at a
    // start (or re-start), never mid-run.
    const int32_t err = target - _actual_pulses;
    const int32_t abs_err = abs(err);
    const int32_t arrive = MAX(pos_db.get(), (int32_t)800);
    const int32_t fast_coast = fast_v / 10 + fast_v * decel_ms / 2000;
    const int16_t creep_rpm_cap = (int16_t)MAX((int16_t)FOLLOW_MIN_RPM, (int16_t)(abs_err * 60 / 6000));
    const int32_t crawl_v = (int32_t)MIN((int16_t)run_speed_rpm(), creep_rpm_cap) * CL57R_STEPS_PER_REV / 60;
    const int32_t crawl_coast = MAX((int32_t)arrive, crawl_v / 10 + crawl_v * decel_ms / 2000);

    if (_follow_moving) {
        const int8_t want_sign = (err > 0) ? 1 : -1;
        if (want_sign != _follow_sign) {
            // Soft stop before reversing.
            stop_follow_leg(now);
            return;
        }
        if (_follow_fast) {
            if (abs_err <= fast_coast) {
                stop_follow_leg(now);
                return;
            }
        } else if (abs_err <= crawl_coast) {
            stop_follow_leg(now);
            return;
        }
        if ((now - _follow_progress_ms) > FOLLOW_REASSERT_MS) {
            _follow_progress_ms = now;
            const int32_t moved = abs(_actual_pulses - _follow_last_enc);
            _follow_last_enc = _actual_pulses;
            if (moved < FOLLOW_MIN_MOVE_PULSES) {
                // Drive dropped speed mode — reassert without reversing.
                send_ff(REG_MOTION, MOTION_SPEED);
            }
        }
        bus_idle_poll(now);
        return;
    }

    if (abs_err <= arrive) {
        bus_idle_poll(now);
        return;
    }

    // Start a leg: full speed only while far enough to coast in safely,
    // otherwise creep at ~0.4*err/s (capped by MAX_SPD). The speed goes out
    // as an awaited write — a silently lost MAX_SPD would leave the previous
    // (possibly much higher) speed in effect, and this drive ignores speed
    // changes once speed mode runs. The SPEED trigger fires only after the
    // drive has confirmed the register (see _follow_speed_pending above).
    const int8_t want_sign = (err > 0) ? 1 : -1;
    int16_t rpm;
    if (abs_err > fast_coast * 2) {
        rpm = (int16_t)run_speed_rpm();
        _follow_fast = true;
    } else {
        rpm = creep_rpm_cap;
        rpm = (int16_t)MIN((int16_t)run_speed_rpm(), rpm);
        _follow_fast = false;
    }
    _follow_last_rpm = (int16_t)(rpm * want_sign);
    _follow_sign = want_sign;
    _follow_progress_ms = now;
    _follow_last_enc = _actual_pulses;
    _follow_speed_pending = true;
    send_u16(REG_MAX_SPD, (uint16_t)_follow_last_rpm);
}

void AP_ModbusSteering::service(uint32_t now)
{
    if (_phase == Phase::IDLE_UNCAL) {
        bus_idle_poll(now);
        return;
    }

    if (_phase == Phase::CAL_ALARM_WAIT) {
        // Echoes here belong to the AUX clear retry, not a phase advance.
        evaluate_cal_alarm_wait(now);
        return;
    }

    if (_phase == Phase::CAL_WAIT ||
        _phase == Phase::MID_FOLLOW ||
        _phase == Phase::BACK_POLL) {
        bus_idle_poll(now);
        if (_phase == Phase::CAL_WAIT) {
            evaluate_cal_wait(now);
        } else if (_phase == Phase::MID_FOLLOW) {
            evaluate_mid_follow(now);
        } else {
            evaluate_back_poll(now);
        }
        return;
    }

    if (_phase == Phase::MID_ZERO) {
        // Halt first, then AUX pos-zero only once the running bit drops;
        // unlike the echo-driven phases this one waits on the status poll.
        if (_got_echo) {
            advance_after_echo();
            return;
        }
        evaluate_mid_zero(now);
        return;
    }

    if (_got_echo) {
        advance_after_echo();
        return;
    }

    if (_got_exception) {
        // The slave rejected the last write. In cal this is fatal for the
        // leg; elsewhere the phase is simply resent on the next tick.
        _got_exception = false;
        _awaiting_echo = false;
        _init_attempts = 0;
        if (in_cal()) {
            abort_home("Modbus exception", false);
            return;
        }
        if (in_run() && _follow_speed_pending) {
            // Slave rejected the MAX_SPD write; re-arm it.
            send_u16(REG_MAX_SPD, (uint16_t)_follow_last_rpm);
        }
        return;
    }

    if (_awaiting_echo) {
        if ((now - _last_send_ms) < ECHO_TIMEOUT_MS) {
            return;
        }
        _init_attempts++;
        _awaiting_echo = false;
        if (_init_attempts >= INIT_MAX_ATTEMPTS) {
            if (in_cal()) {
                abort_home("no Modbus reply", false);
                return;
            }
            if (in_run()) {
                _phase = Phase::IDLE_READY;
                _init_attempts = 0;
                if (_follow_speed_pending) {
                    // The MAX_SPD write never came back; re-arm it instead
                    // of letting the SPEED trigger fire at a stale speed.
                    send_u16(REG_MAX_SPD, (uint16_t)_follow_last_rpm);
                }
                return;
            }
            static uint32_t last_fail_ms;
            if (last_fail_ms == 0 || (now - last_fail_ms) > 5000) {
                last_fail_ms = now;
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "CL57R: init retry phase=%u",
                              (unsigned)_phase);
            }
            _init_attempts = 0;
        }
    }

    if (_phase == Phase::IDLE_READY) {
        return;
    }

    if (_last_send_ms != 0 && (now - _last_send_ms) < SEND_INTERVAL_MS) {
        return;
    }
    send_current();
}

bool AP_ModbusSteering::rc_override_active() const
{
    for (uint8_t i = 0; i < 16; i++) {
        const RC_Channel *chan = rc().channel(i);
        if (chan != nullptr && chan->has_override()) {
            return true;
        }
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
        if (!was_high) {
            was_high = true;
            return true;
        }
        return false;
    }
    was_high = false;
    return false;
}

void AP_ModbusSteering::poll_triggers()
{
    // RC override path is used by GCS joystick and bench scripts; it must
    // work whenever the library is live, not only in MANUAL mode.
    if (rc_override_active()) {
        _rc_override_seen = true;
    }
    if (_rc_override_seen) {
        rc_rising_edge(cal_ch.get(), _cal_was_high);
        rc_rising_edge(rst_ch.get(), _rst_was_high);
    } else {
        if (rc_rising_edge(cal_ch.get(), _cal_was_high)) {
            start_home();
        }

        // Alarm clear button: only meaningful once init is done.
        if ((_phase == Phase::IDLE_UNCAL || _phase == Phase::IDLE_READY) &&
            rc_rising_edge(rst_ch.get(), _rst_was_high)) {
            send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: alarm clear (RC)");
        }
    }

    const int8_t trig = cal_trig.get();
    if (!_cal_trig_inited) {
        _cal_trig_last = trig;
        _cal_trig_inited = true;
        return;
    }
    if (trig == _cal_trig_last) {
        return;
    }
    _cal_trig_last = trig;
    if (trig == 1) {
        start_home();
    } else if (trig == 2) {
        if (hal.util->get_soft_armed()) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "CL57R: CAL_TRIG ignored, armed");
            clear_cal_trig();
            return;
        }
        send_u16(REG_AUX_CONTROL, AUX_ALARM_CLEAR);
        clear_cal_trig();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "CL57R: alarm clear (TRIG)");
    } else if (trig != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "CL57R: invalid CAL_TRIG %d", (int)trig);
        clear_cal_trig();
    }
}

void AP_ModbusSteering::update(float steering_out)
{
    if (_uart == nullptr) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    consume_rx();
    poll_triggers();
    if (_homed && in_run()) {
        handle_run(now, steering_out);
    }
    service(now);
}
