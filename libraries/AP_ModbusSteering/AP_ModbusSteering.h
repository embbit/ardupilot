#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_ModbusSteering {
public:
    AP_ModbusSteering();

    void init(AP_SerialManager &serial_manager);
    void update(float steering_out);

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
    };

    int32_t travel_limit_pulses() const;
    uint8_t rtu_frame_len(const uint8_t *buf, uint8_t avail) const;
    void consume_rx();
    void advance_init();

    AP_HAL::UARTDriver *_uart = nullptr;
    uint32_t _last_send_ms = 0;
    uint32_t _last_vect_ms = 0;
    uint32_t _last_rx_ms = 0;
    DriveState _state = DriveState::INIT_ENABLE;
    uint8_t _init_attempts = 0;
    bool _got_echo = false;
    bool _have_target = false;
    int32_t _last_target = 0;
    int32_t _actual_pulses = 0;
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
};
