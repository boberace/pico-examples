#include "pid.h"
#include<math.h>

pid::pid(float kp, float ki, float kd) {
    _setpoint = 0; // value you want the controller output to maintain
    _kp = kp; // proportional constant of PID
    _ki = ki; // integral constant of PID
    _kd = kd; // derivative constant of PID

    _difference_error = 0; //  error between the setpoint and the sensor 
    _accumulative_error = 0; //  accumulation of difference error proportional to the time change
    _controller_output = 0; // arbitrated output based on PID, deaband, and minimum output relationships

}


float pid::output_update(float sensor_input, float time_change) {

    float previous_difference_error, previous_controller_output, pid_adjustment;

    previous_controller_output = _controller_output;
    previous_difference_error = _difference_error;
 
    _difference_error = _setpoint - sensor_input; 
    // The following if statment prevents the controller from accumulating error at maximum controller output (aka integral windup)
    if ((previous_controller_output > -1) and (previous_controller_output < 1)) _accumulative_error += _difference_error*time_change; 

    _rate_error = (_difference_error - previous_difference_error )/time_change;    

    pid_adjustment = _kp*_difference_error + _ki*_accumulative_error + _kd*_rate_error;

    _controller_output = previous_controller_output + pid_adjustment;

    return _controller_output > 1 ? 1 : (_controller_output < -1 ? -1 : _controller_output);

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

