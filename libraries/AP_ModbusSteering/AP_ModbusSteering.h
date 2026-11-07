#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_ModbusSteering {
public:
    AP_ModbusSteering();

    void init(AP_SerialManager &serial_manager);
    // steering_out is -1..+1. After calibration, maps to ±OB_STR_MAX_STEPS.
    void update(float steering_out);

    bool enabled() const { return _uart != nullptr; }
    bool homed() const { return _homed; }
    bool homing() const;
    bool alarmed() const { return (_status_word & (1U << 3)) != 0; }
    bool link_ok() const;

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class Phase : uint8_t {
        INIT_ENABLE = 0,
        INIT_CLEAR,
        INIT_X1,
        INIT_X2,
        INIT_POL,
        IDLE,

        SEEK_STOP,
        SEEK_CLEAR,
        SEEK_DISABLE,
        SEEK_ENABLE,
        SEEK_X1,
        SEEK_X2,
        SEEK_POL,
        SEEK_STARTSPD,
        SEEK_ACCEL,
        SEEK_DECEL,
        SEEK_MAXSPD,
        SEEK_TARGET,
        SEEK_RUN,
        SEEK_POLL,
        SEEK_HIT_STOP,
        SEEK_HIT_WAIT,

        BACK_STOP,
        BACK_CLEAR,
        BACK_ENABLE,
        BACK_SPD,
        BACK_START,
        BACK_POLL,
        BACK_HALT,

        MID_STOP,
        MID_CLEAR,
        MID_ENABLE,
        MID_STARTSPD,
        MID_ACCEL,
        MID_DECEL,
        MID_MAXSPD,
        MID_TARGET,
        MID_RUN,
        MID_POLL,
        MID_ZERO,

        FOLLOW_STOP,
        FOLLOW_STARTSPD,
        FOLLOW_ACCEL,
        FOLLOW_DECEL,
        FOLLOW_MAXSPD,
        FOLLOW_TARGET,
        FOLLOW_RUN,

        ABORT_STOP,
        ALARM_CLEAR,
        ALARM_ENABLE,
    };

    enum class RxExpect : uint8_t {
        NONE = 0,
        WRITE,
        STATUS,
        DI,
        ENCODER,
    };

    bool in_init() const { return _phase <= Phase::INIT_POL; }
    bool seeking() const { return _phase >= Phase::SEEK_STOP && _phase <= Phase::MID_ZERO; }
    bool following() const { return _phase >= Phase::FOLLOW_STOP && _phase <= Phase::FOLLOW_RUN; }

    uint32_t link_timeout_ms() const;
    uint32_t home_timeout_ms() const;
    uint16_t home_rpm() const;
    uint16_t crawl_rpm() const;
    uint16_t seek_rpm() const;
    uint16_t follow_rpm() const;
    int16_t backoff_rpm() const;
    int32_t expected_full_travel() const;
    int32_t stick_target_pulses() const;
    uint16_t polarity_word() const;
    bool limit_active() const;
    bool both_limits() const;
    bool x1_active() const;
    bool x2_active() const;
    uint16_t own_limit_bit() const;

    void enter(Phase phase);
    void start_home();
    void abort_home(const char *reason, bool estop = false);
    void begin_backoff(const char *why);
    void begin_hard_stop_recovery();
    void mark_end(bool falling_proxy);
    void complete_end();
    void finish_mid_zero();
    void on_reply(uint32_t now);
    void advance_setup();
    void evaluate_seek(uint32_t now);
    void evaluate_back(uint32_t now);
    void evaluate_mid(uint32_t now);
    void evaluate_follow_guard();
    void soft_follow_ot();
    void finish_to_idle();
    void poll_triggers();
    bool try_begin_follow();
    bool rc_rising_edge(int8_t ch, bool &was_high) const;

    void send_current();
    void send_u16(uint16_t reg, uint16_t value);
    void send_s16(uint16_t reg, int16_t value);
    void send_read(uint16_t reg, uint16_t count, RxExpect expect);
    void send_target(int32_t target);
    void arm_wait(RxExpect expect);
    bool take_reply();
    void consume_rx();
    uint8_t rtu_frame_len(const uint8_t *buf, uint8_t avail) const;
    void note_encoder(int32_t enc, uint32_t now);
    void note_di(uint16_t raw);

    AP_HAL::UARTDriver *_uart = nullptr;
    Phase _phase = Phase::INIT_ENABLE;
    RxExpect _expect = RxExpect::NONE;
    uint8_t _rx_buf[64] {};
    uint8_t _rx_len = 0;
    uint8_t _tx_tries = 0;
    uint8_t _poll_step = 0;
    uint8_t _leg = 0;
    uint8_t _leg_retries = 0;
    uint16_t _method = 17;
    uint16_t _status_word = 0;
    uint16_t _fault_code = 0;
    uint16_t _di_word = 0;
    uint16_t _di_raw = 0;
    uint16_t _di_log_raw = 0;
    uint16_t _di_log_eff = 0;
    bool _di_log_inited = false;
    bool _waiting = false;
    bool _got_echo = false;
    bool _got_status = false;
    bool _got_di = false;
    bool _got_encoder = false;
    bool _homed = false;
    bool _home_pending = false;
    bool _ever_rx = false;
    bool _di_ready = false;
    bool _limit_at_start = false;
    bool _saw_limit = false;
    bool _saw_motion = false;
    bool _saw_home_clear = false;
    bool _back_from_limit = false;
    bool _use_estop = false;
    bool _in_recovery = false;
    bool _end_from_recovery = false;
    bool _recovery_saw_active = false;
    bool _limit_was_active = false;
    uint8_t _limit_hit_count = 0;
    uint8_t _recovery_used = 0;
    uint16_t _recovery_bit = 0;
    uint16_t _failed_method = 17;
    bool _rst_was_high = false;
    bool _cal_was_high = false;
    bool _cal_trig_inited = false;
    int8_t _cal_trig_last = 0;
    int32_t _actual_pulses = 0;
    int32_t _seek_origin = 0;
    int32_t _back_origin = 0;
    int32_t _leg_peak = 0;
    int32_t _end_pos[2] {};
    int32_t _hard_stop_pos = 0;
    int32_t _recovery_origin = 0;
    int32_t _mid_target = 0;      // absolute mid before zero
    int32_t _mid_rel = 0;         // relative crawl step issued for mid
    int32_t _travel_pulses = 0;
    float _stick = 0.0f;
    int32_t _follow_target = 0;
    int32_t _follow_cmd_target = 0;
    bool _follow_profile_ok = false;
    bool _follow_force = false;
    bool _follow_via_stop = false;
    bool _follow_limit_hold = false; // STOP then IDLE; do not retarget
    bool _await_stick_center = false;
    bool _mid_alarm_retried = false;
    uint8_t _mid_rel_retries = 0; // extra REL hops to actually reach mid
    bool _keep_seek_origin = false; // early-OT crawl-on: do not reset leg origin
    uint32_t _last_send_ms = 0;
    uint32_t _wait_ms = 0;
    uint32_t _phase_ms = 0;
    uint32_t _leg_start_ms = 0;
    uint32_t _last_rx_ms = 0;
    uint32_t _last_motion_ms = 0;
    uint32_t _last_button_ms = 0;
    uint32_t _last_init_warn_ms = 0;

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
};
