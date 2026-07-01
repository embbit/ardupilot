#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_SerialManager/AP_SerialManager.h>

class AP_ModbusSteering {
public:
    AP_ModbusSteering();

    // Инициализация порта
    void init(AP_SerialManager &serial_manager);

    // Основной цикл управления, вызываемый из Ardupilot
    void update(float steering_out);

    // Описание параметров для Mission Planner
    static const AP_Param::GroupInfo var_info[];

private:
    AP_HAL::UARTDriver *_uart = nullptr;
    uint32_t _last_send_ms = 0;
    const uint32_t SEND_INTERVAL_MS = 100; // Частота работы шины 10 Гц (100 мс)

    // Параметры Ardupilot
    AP_Int8  slave_id;     // ID драйвера CL57R в сети Modbus
    AP_Int16 reg_address;   // Базовый регистр позиции
    AP_Int32 max_steps;     // Максимальный рабочий диапазон руля в импульсах
    AP_Int16 start_speed;   // Стартовая скорость JOG (Регистр 0x0030)
    AP_Int16 max_speed;     // Максимальная рабочая скорость в об/мин (Регистр 0x0033)
};
