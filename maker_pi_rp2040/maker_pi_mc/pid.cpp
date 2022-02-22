#include "pid.h"

pid::pid(float kp, float ki, float kd) {
    _setpoint = 0;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _previous_difference_error = 0;
    _accumulative_error = 0;
    _controller_output = 0;

}

float pid::output_update(float sensor_input, float time_change) {

    float difference_error, rate_error;
 
    difference_error = _setpoint - sensor_input; 
    // The if statment prevents the controller from accumulating error at maximum controller output (AKA preventing integral windup)
    if (_controller_output > -1 and _controller_output < 1) _accumulative_error += difference_error*time_change; 
    rate_error = (difference_error - _previous_difference_error )/time_change;
    _previous_difference_error = difference_error;

    _controller_output = _kp*difference_error + _ki*_accumulative_error + _kd*rate_error;    

    return _controller_output;

} 

void pid::set_setpoint(float sp){
    _setpoint = sp;
}

float pid::get_setpoint(){
    return _setpoint;
}

void pid::set_kp(float kp){
    _kp = kp;
}

void pid::set_ki(float ki){
    _ki = ki;
}

void pid::set_kd(float kd){
    _kd = kd;
}

float pid::get_kp(){
    return _kp;
}

float pid::get_ki(){
    return _ki;
}

float pid::get_kd(){
    return _kd;
}