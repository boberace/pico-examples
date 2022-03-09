#include "pid.h"
#include<math.h>

pid::pid(float kp, float ki, float kd, float db, float mo) {
    _setpoint = 0; // value you want the controller output to maintain
    _kp = kp; // proportional constant of PID
    _ki = ki; // integral constant of PID
    _kd = kd; // derivative constant of PID
    _db = db; // dead band - prevents small fluctions in output
    // some equipment, typically motor driven, will be damaged if driven below a specific threshold
    _mo = mo; // minimum output - set the instance to the minimum threshold - motors typically around 0.2
    _difference_error = 0; //  error between the setpoint and the sensor 
    _accumulative_error = 0; //  accumulation of difference error proportional to the time change
    _controller_output = 0; // arbitrated output based on PID, deaband, and minimum output relationships

}

float pid::output_update(float sensor_input, float time_change) {

    float previous_difference_error, previous_controller_output, pid_output;

    previous_controller_output = _controller_output;
    previous_difference_error = _difference_error;
 
    _difference_error = _setpoint - sensor_input; 
    // The following if statment prevents the controller from accumulating error at maximum controller output (aka integral windup)
    if (previous_controller_output > -1 and previous_controller_output < 1) _accumulative_error += _difference_error*time_change; 
    _rate_error = (_difference_error - previous_difference_error )/time_change;    

    pid_output = _kp*_difference_error + _ki*_accumulative_error + _kd*_rate_error;

    // this is where the deadband and minimum output works.  The absolute difference between the current and previous outputs 
    // must be greater than the deadband and the output must be greater than
    if (abs(pid_output - previous_controller_output) > _db and pid_output > _mo)  _controller_output = pid_output;
    else if (pid_output < _mo) _controller_output = 0;
    else  _controller_output = previous_controller_output ;

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

void pid::set_db(float db){
    _db = db;
}

void pid::set_mo(float mo){
    _mo = mo;
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

float pid::get_db(){
    return _db;
}

float pid::get_mo(){
    return _mo;
}