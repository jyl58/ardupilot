#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

class AP_Proximity_DYP:public AP_Proximity_Backend
{
    public:
        AP_Proximity_DYP(AP_Proximity &_frontend,AP_Proximity::Proximity_State &_state);
        
        static bool detect();
        void update(void)override;
         // get maximum and minimum distances (in meters) of sensor
        float distance_max() const override { return 4.5f; }
        float distance_min() const override { return 0.30f; }
    private:
        void initialise();
        uint32_t _last_distance_received_ms;    // system time of last distance measurement received from sensor
        AP_HAL::UARTDriver *_uart=nullptr;              // uart for communicating with sensor

};