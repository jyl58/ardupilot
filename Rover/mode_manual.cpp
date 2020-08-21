#include "mode.h"
#include "Rover.h"

bool ModeManual::_enter()
{
    _try_control=false;
    _try_start_time_ms=0;
    return true;
}
void ModeManual::_exit()
{
    // clear lateral when exiting manual mode
    g2.motors.set_lateral(0);
    g2.serial_control.setMotorControlMode(SerialControl::MotorRunMode_None);
    _try_control=false;
    _try_start_time_ms=0;
}

void ModeManual::update()
{
    float desired_steering, desired_throttle;// desired_lateral;
    get_pilot_desired_steering_and_throttle(desired_steering, desired_throttle);
    //get_pilot_desired_lateral(desired_lateral);

    // if vehicle is balance bot, calculate actual throttle required for balancing
    /*if (rover.is_balancebot()) {
        rover.balancebot_pitch_control(desired_throttle);
    }*/
    g2.serial_control.setMotorControlMode(SerialControl::MotorRunMode_manual);
    //try the motor
    if(is_zero(desired_throttle)){
        if(_try_control){
            if(_try_start_time_ms==0){
                _try_start_time_ms=AP_HAL::millis();
            }
            
            if(AP_HAL::millis()-_try_start_time_ms<3000){
                desired_throttle=10.0f;
            }else if((AP_HAL::millis()-_try_start_time_ms>3000)&&(AP_HAL::millis()-_try_start_time_ms<6000)){
                desired_throttle=-10.0f;
            }else{
                _try_control=false;
                _try_start_time_ms=0;
            }
        }
    }else{
        _try_control=false;
        _try_start_time_ms=0;
    }
    // set sailboat sails
    //float desired_mainsail;
    //float desired_wingsail;
    //g2.sailboat.get_pilot_desired_mainsail(desired_mainsail, desired_wingsail);
    //g2.motors.set_mainsail(desired_mainsail);
    //g2.motors.set_wingsail(desired_wingsail);

    // copy RC scaled inputs to outputs
    g2.motors.set_throttle(desired_throttle);
    g2.motors.set_steering(desired_steering, false);
    //g2.motors.set_lateral(desired_lateral);
}
