#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

/*
 * CL57R Modbus RTU outboard steering (Rover OB_STR_*).
 *
 * Phases: Init → IdleUncal → Cal (dual MOTION_HOME + mid zero) →
 * IdleReady / absolute Run (stick or autopilot) with link failsafe.
 */
class AP_ModbusSteering {
public:
    AP_ModbusSteering();

    void init(AP_SerialManager &serial_manager);
    void update(float steering_out);

    bool enabled() const { return _uart != nullptr; }
    bool homed() const { return _homed; }
    bool homing() const;
    bool alarmed() const { return (_status_word & (1U << 3)) != 0; }
    bool link_ok() const;
    int32_t debug_enc() const { return _actual_pulses; }
    uint32_t debug_status() const { return (uint32_t)(_status_word | (_err_code << 8)); }
    uint32_t debug_di() const { return (uint32_t)_di_word; }
    int32_t debug_follow_target() const { return _debug_target; }
    uint32_t debug_txstate(uint32_t now) const
    {
        (void)now;
        return (uint32_t)(_awaiting_echo ? 1 : 0) |
               (_got_echo ? 2 : 0) |
               (_got_exception ? 4 : 0) |
               ((uint32_t)_rx_expect << 3) |
               ((uint32_t)_init_attempts << 8) |
               ((uint32_t)_phase << 16) |
               ((uint32_t)(_follow_moving ? 1 : 0) << 24) |
               ((uint32_t)(_follow_sign & 0x3) << 25) |
               ((uint32_t)(_run_limit_hold ? 1 : 0) << 27) |
               ((uint32_t)(_follow_fast ? 1 : 0) << 28);
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class Phase : uint8_t {
        INIT_ENABLE = 0,
        INIT_CLEAR_ALARM,
        INIT_SUBDIVISION,
        INIT_START_SPD,
        INIT_MAX_SPD,
        INIT_ACCEL,
        INIT_DECEL,
        INIT_ABS_MODE,
        INIT_TRACK_ERR,
        IDLE_UNCAL,

        // Recover from a latched drive alarm before touching any motion
        // register: this unit ignores starts while the alarm bit is set.
        CAL_ALARM_WAIT,
        CAL_STOP,
        CAL_CLEAR,
        CAL_ENABLE,
        CAL_METHOD,
        CAL_HOME_SPD,
        CAL_CRAWL,
        CAL_HOME_ACCEL,
        CAL_START_HOME,
        CAL_WAIT,

        MID_STOP,
        MID_CLEAR,
        MID_ENABLE,
        MID_MAX_SPD,
        MID_SPD,
        MID_GO,
        MID_FOLLOW,
        MID_HALT,
        MID_ZERO,

        BACK_STOP,
        BACK_CLEAR,
        BACK_ENABLE,
        BACK_SPD,
        BACK_START,
        BACK_POLL,

        ABORT_STOP,

        IDLE_READY,
        RUN_STOP,

        // Post-cal: restore armed run speed in REG_MAX_SPD before IDLE_READY.
        IDLE_SET_SPD,
    };

    enum class RxExpect : uint8_t {
        NONE = 0,
        ENCODER,
        STATUS,
        DI_INPUT,
    };

    bool in_init() const { return _phase <= Phase::INIT_TRACK_ERR; }
    bool in_cal() const {
        return _phase >= Phase::CAL_ALARM_WAIT && _phase <= Phase::ABORT_STOP;
    }
    bool in_run() const {
        return _phase == Phase::IDLE_READY ||
               _phase == Phase::RUN_STOP;
    }

    uint8_t rtu_frame_len(const uint8_t *buf, uint8_t avail) const;
    void consume_rx();
    uint32_t link_timeout_ms() const;
    uint32_t home_timeout_ms() const;
    uint16_t run_speed_rpm() const;
    uint16_t home_seek_rpm() const;
    uint16_t home_crawl_rpm() const;
    uint16_t home_first_method() const;
    uint16_t home_method_now() const;
    uint16_t home_limit_bit() const;
    uint16_t limit_bit_for_method(uint16_t method) const;
    int16_t mid_cruise_rpm() const;
    int32_t mid_brake_pulses() const;
    int32_t mid_final_pulses() const;
    int32_t expected_full_travel() const;
    int32_t travel_limit_pulses() const;
    int16_t backoff_signed_rpm() const;

    void send_u16(uint16_t reg, uint16_t value);
    void write_all(const uint8_t *buf, uint16_t len);
    void send_ff(uint16_t reg, uint16_t value);
    void send_read(uint16_t reg, uint16_t count, RxExpect expect);
    void send_current();

    void advance_after_echo();
    void service(uint32_t now);
    void bus_idle_poll(uint32_t now);
    void evaluate_cal_wait(uint32_t now);
    void evaluate_cal_alarm_wait(uint32_t now);
    void evaluate_mid_follow(uint32_t now);
    void evaluate_mid_zero(uint32_t now);
    void evaluate_back_poll(uint32_t now);
    void handle_run(uint32_t now, float steering_out);
    void handle_run_link_failsafe(uint32_t now);

    void start_home();
    void begin_leg_prep();
    void begin_mid();
    void begin_backoff(const char *why);
    void abort_home(const char *reason, bool estop);
    void finish_cal();
    void clear_cal_trig();

    void poll_triggers();
    bool rc_rising_edge(int8_t ch, bool &was_high) const;
    bool rc_override_active() const;

    AP_HAL::UARTDriver *_uart = nullptr;
    uint32_t _last_send_ms = 0;
    uint32_t _last_rx_ms = 0;
    uint32_t _rx_crc_err = 0;
    uint32_t _last_read_sent_ms = 0;
    uint32_t _cal_start_ms = 0;
    uint32_t _leg_start_ms = 0;
    uint32_t _last_progress_ms = 0;
    uint32_t _back_start_ms = 0;

    Phase _phase = Phase::INIT_ENABLE;
    RxExpect _rx_expect = RxExpect::NONE;
    uint8_t _init_attempts = 0;
    bool _awaiting_echo = false;
    bool _got_echo = false;
    bool _got_exception = false;
    bool _homed = false;
    bool _got_status = false;
    bool _got_di = false;
    bool _saw_home_clear = false;
    bool _saw_home_run = false;
    bool _saw_limit = false;
    bool _use_estop = false;
    uint8_t _home_leg = 0;       // 0=first, 1=second
    uint8_t _back_tries = 0;
    uint8_t _run_poll_phase = 0;
    uint16_t _di_word = 0;
    uint16_t _status_word = 0;
    uint16_t _err_code = 0;
    int32_t _actual_pulses = 0;
    int32_t _last_progress_enc = 0;
    int32_t _leg1_travel = 0;
    int32_t _leg_peak_travel = 0;
    int32_t _measured_half_travel = 0;
    int32_t _mid_target = 0;
    int8_t _leg_dir = 0;         // observed direction of the current seek
    int32_t _mid_origin_enc = 0;
    int32_t _mid_last_enc = 0;
    int32_t _mid_traveled = 0;
    int8_t _mid_dir = 0;
    bool _mid_slow = false;
    bool _mid_zero_sent = false;
    bool _cal_alarm_stop_next = false;
    uint8_t _pos_end_bit = 0x02;  // limit input at the encoder-positive end (set by cal)
    uint8_t _neg_end_bit = 0x04;  // limit input at the encoder-negative end (set by cal)
    int8_t _cal_trig_last = 0;
    bool _cal_trig_inited = false;
    bool _rc_override_seen = false;
    bool _rst_was_high = false;
    bool _cal_was_high = false;
    bool _run_link_failsafe = false;
    bool _run_limit_hold = false;
    bool _follow_moving = false;
    bool _follow_fast = false;
    bool _follow_speed_pending = false;
    int8_t _follow_sign = 0;
    int16_t _follow_last_rpm = 0;
    uint32_t _follow_progress_ms = 0;
    int32_t _follow_last_enc = 0;
    int32_t _debug_target = 0;
    uint32_t _link_lost_candidate_ms = 0;
    uint32_t _last_run_link_warn_ms = 0;
    uint8_t _rx_buf[64] {};
    uint8_t _rx_len = 0;

    AP_Int8  slave_id;
    AP_Int16 reg_address;
    AP_Int32 max_steps;
    AP_Int16 start_speed;
    AP_Int16 max_speed;
    AP_Int32 pos_db;
    AP_Int32 ret_slew;
    AP_Int8  out_rev;
    AP_Int16 ratio;
    AP_Int8  rst_ch;
    AP_Int8  cal_ch;
    AP_Int8  cal_mth;
    AP_Int16 home_speed;
    AP_Int8  cal_trig;
    AP_Int8  cal_mode;
    AP_Int16 crawl_speed;
    AP_Int16 link_timeout;
    AP_Int16 cal_timeout_s;
    AP_Int8  di_inv;
    AP_Int32 track_err_limit;
    AP_Int16 acc_ms;
};
