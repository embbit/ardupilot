#ifndef AP_FAR_DRIVER_THROTTLE_H
#define AP_FAR_DRIVER_THROTTLE_H

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Param/AP_Param.h>
#include "fardriver_protocol.h"

class AP_FarDriverThrottle 
{
    public:
        AP_FarDriverThrottle();

        void init(AP_SerialManager &serial_manager);
        void update(float throttle_out);

        static const struct AP_Param::GroupInfo var_info[];

    private:
        AP_HAL::UARTDriver* _uart = nullptr;
        uint32_t _last_send_ms = 0;
        const uint32_t SEND_INTERVAL_MS = 100; 

        AP_Int16 max_rpm;
};

#endif
