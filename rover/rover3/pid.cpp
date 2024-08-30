#include "pid.h"

pid::pid(float kp, float ki, float kd) {

    _setpoint = 0; 

    _kp = kp;
    _ki = ki; 
    _kd = kd; 

    _error_integral = 0; 
    _error_previous = 0;
    _controller_output = 0;

}


float pid::output_update(float sensor_input, float time_change) {

    float error, error_derivative;
 
    error = _setpoint - sensor_input; 
    _error_previous = error;

    // The following if statement prevents integration of error at maximum controller output (aka integral windup)
    if (((_controller_output = -1) and (error > 0)) or ((_controller_output = 1) and (error < 0)) )
    _error_integral += error*time_change; 

    error_derivative = (error - _error_previous )/time_change;    

    _controller_output = _kp*error + _ki*_error_integral + _kd*error_derivative;     

    // The following if statements prevent the controller output from exceeding the limits of -1 to 1
    if (_controller_output > 1) _controller_output = 1;
    if (_controller_output < -1) _controller_output = -1;       
    
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

