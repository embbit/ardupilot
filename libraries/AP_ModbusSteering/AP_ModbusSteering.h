#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_ModbusSteering {
public:
    AP_ModbusSteering();

    // Инициализация порта
    void init(AP_SerialManager &serial_manager);

    // Основной цикл управления
    void update(float steering_out);

    // Описание параметров для Mission Planner
    static const AP_Param::GroupInfo var_info[];

private:
    AP_HAL::UARTDriver *_uart = nullptr;
    uint32_t _last_send_ms = 0;

    // Параметры Ardupilot
    AP_Int8  slave_id;     
    AP_Int16 reg_address;   
    AP_Int32 max_steps;     
    AP_Int16 start_speed;   
    AP_Int16 max_speed;
    AP_Int32 pos_db;
    AP_Int32 ret_slew;
    AP_Int8  out_rev;
    AP_Int16 ratio;

    int32_t travel_limit_pulses() const;
};
