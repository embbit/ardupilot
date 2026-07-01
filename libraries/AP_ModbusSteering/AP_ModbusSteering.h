#ifndef AP_MODBUS_STEERING_H
#define AP_MODBUS_STEERING_H

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>
#include "modbus_protocol.h"

class AP_ModbusSteering {
public:
    AP_ModbusSteering();

    void init(AP_SerialManager &serial_manager);
    void update(float steering_out); 

    static const struct AP_Param::GroupInfo var_info[];
    
private:
    AP_HAL::UARTDriver* _uart = nullptr;
    uint32_t _last_send_ms = 0;
    const uint32_t SEND_INTERVAL_MS = 20; 

    AP_Int8  slave_id;    
    AP_Int16 reg_address; 
    AP_Int16 max_steps;   
};

#endif

