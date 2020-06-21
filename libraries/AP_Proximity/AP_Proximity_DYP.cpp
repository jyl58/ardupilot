#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_Math/crc.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include "AP_Proximity_DYP.h"
#define PROXIMITY_DYP_TIMEOUT_MS 200
extern const AP_HAL::HAL& hal;

AP_Proximity_DYP::AP_Proximity_DYP(AP_Proximity &_frontend,AP_Proximity::Proximity_State &_state)
:AP_Proximity_Backend(_frontend, _state)
{
    const AP_SerialManager &serial_manager = AP::serialmanager();
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Proximity_DYP, 0);
    if (_uart != nullptr) {
        // start uart with larger receive buffer
        _uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Proximity_DYP, 0), 512, 0);
    }
    initialise();
}
bool 
AP_Proximity_DYP::detect()
{
    return AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_Proximity_DYP, 0)!=nullptr;
}
void 
AP_Proximity_DYP::initialise()
{
    // initialise boundary
    init_boundary();
}
void 
AP_Proximity_DYP::update()
{
    if (_uart == nullptr) {
        return;
    }
    //TODO: add the read uart code

    if ((_last_distance_received_ms == 0) || ((AP_HAL::millis() - _last_distance_received_ms) > PROXIMITY_DYP_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}