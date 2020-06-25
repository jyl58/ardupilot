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
    int16_t nbytes = _uart->available();
    while (nbytes-- > 0) {
        const int16_t r = _uart->read();
        if ((r < 0) || (r > 0xFF)) {
            continue;
        }
        collect_byte((uint8_t)r);
    }
    if ((_last_distance_received_ms == 0) || ((AP_HAL::millis() - _last_distance_received_ms) > PROXIMITY_DYP_TIMEOUT_MS)) {
        set_status(AP_Proximity::Status::NoData);
    } else {
        set_status(AP_Proximity::Status::Good);
    }
}
void 
AP_Proximity_DYP::collect_byte(uint8_t msg)
{
    switch(_parse_phase){
        case  STEP_head:
            if(msg==0XFF){
                _parse_phase=STEP_data;
                _data_index=0;
            }
            break;
        case STEP_data:
            _data[_data_index]=msg;
            _data_index++;
            if(_data_index>=8){
                parserMsg();
                _parse_phase=STEP_data;
            }
            break;
        default :
        break;
    }
}
void
AP_Proximity_DYP::parserMsg()
{
    uint16_t check_sum=0XFF;
    for(int i=0;i<8;i++){
        check_sum+=_data[i];
    }
    uint8_t low_check_sum=(uint8_t) check_sum&0xFF;
    if(low_check_sum==_data[8]){
        uint32_t now = AP_HAL::millis();
        _last_distance_received_ms= now;
        _angle[0] = 315;
        _distance[0] = (_data[0]*256+_data[1])*0.001;//mm-->m
        _distance_valid[0] = (_distance[0] >= 0.3) && (_distance[0] <= 4.5);
        update_boundary_for_sector(0, true);
            
        _angle[1] = 0;
        _distance[1] = (_data[2]*256+_data[3])*0.001;//mm-->m
        _distance_valid[1] = (_distance[1] >= 0.3) && (_distance[1] <= 4.5);
        update_boundary_for_sector(1, true);
    
        _angle[2] = 45;
        _distance[2] = (_data[4]*256+_data[5])*0.001;//mm-->m
        _distance_valid[2] = (_distance[2] >= 0.3) && (_distance[2] <= 4.5);
        update_boundary_for_sector(2, true);

        _angle[3] = 0;
        _distance[3] = (_data[6]*256+_data[7])*0.001;//mm-->m
        _distance_valid[3] = (_distance[3] >= 0.3) && (_distance[3] <= 4.5);
        update_boundary_for_sector(3, true);
    }
}