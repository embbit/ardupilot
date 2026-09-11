#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_ModbusSteering {
public:
    AP_ModbusSteering();

    void init(AP_SerialManager &serial_manager);
    void update(float steering_out);

    bool enabled() const { return _uart != nullptr; }
    bool homed() const { return _homed; }
    bool homing() const { return in_home(); }
    bool alarmed() const { return (_status_word & (1U << 3)) != 0; }

    static const struct AP_Param::GroupInfo var_info[];

private:
    enum class DriveState : uint8_t {
        INIT_ENABLE = 0,
        INIT_CLEAR_ALARM,
        INIT_SUBDIVISION,
        INIT_START_SPD,
        INIT_MAX_SPD,
        INIT_ACCEL,
        INIT_DECEL,
        INIT_ABS_MODE,
        RUN_WRITE,
        RUN_READ,
        HOME_CLEAR_ALARM,
        HOME_SET_METHOD,
        HOME_SET_SPD,
        HOME_SET_RUN_SPD,
        HOME_SET_CRAWL,
        HOME_SET_ACCEL,
        HOME_START,
        HOME_WAIT,
        HOME_MOVE_CENTER,
        HOME_WAIT_CENTER,
        HOME_ZERO,
    };

    enum class RxExpect : uint8_t {
        NONE = 0,
        ENCODER,
        STATUS,
    };

    int32_t travel_limit_pulses() const;
    uint8_t rtu_frame_len(const uint8_t *buf, uint8_t avail) const;
    void consume_rx();
    void advance_init();
    void advance_home();
    void poll_rc_buttons();
    bool rc_rising_edge(int8_t ch, bool &was_high) const;
    void request_alarm_clear();
    void start_home();
    bool in_run() const;
    bool in_home() const;
    bool home_prep_wait_echo() const;
    uint16_t home_method_reg() const;
    uint16_t run_speed_rpm() const;
    uint16_t calib_speed_rpm() const;
    int32_t center_target_pulses() const;
    void send_u16(uint16_t reg, uint16_t value);
    void send_abs_move(int32_t target);
    void finish_home();
    void abort_home(const char *reason);

    AP_HAL::UARTDriver *_uart = nullptr;
    uint32_t _last_send_ms = 0;
    uint32_t _last_vect_ms = 0;
    uint32_t _last_rx_ms = 0;
    uint32_t _home_start_ms = 0;
    uint32_t _last_button_ms = 0;
    DriveState _state = DriveState::INIT_ENABLE;
    RxExpect _rx_expect = RxExpect::NONE;
    uint8_t _init_attempts = 0;
    bool _got_echo = false;
    bool _have_target = false;
    bool _alarm_clear_pending = false;
    bool _pending_run_spd = false;
    bool _home_pending = false;
    bool _homed = false;
    bool _was_armed = false;
    bool _rst_was_high = false;
    bool _home_was_high = false;
    bool _got_status = false;
    bool _saw_home_run = false;
    bool _read_status_next = false;
    int32_t _last_target = 0;
    int32_t _actual_pulses = 0;
    int32_t _center_target = 0;
    uint16_t _status_word = 0;
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
    AP_Int8  home_ch;
    AP_Int8  home_mth;
    AP_Int16 home_speed;
};
