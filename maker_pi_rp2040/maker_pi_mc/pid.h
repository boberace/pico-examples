#ifndef pid_h
#define pid_h
// creates a simple PID instance
// the control setpoint is between -1 to 1
// the sensor input is between -1 to 1
// wikipedia has a very good page on PID controllers
// todo - 
//        create gain schedule to compensate for high and low rate 
//        test standard form
//        create IIR for derivative

class pid {
  public:
    pid(float kp = 0.25, float ki = 0.1, float kd=0.01, float db=0, float mo=0);
    float output_update(float sensor_input, float time_change); 
    void set_setpoint(float sp);
    float get_setpoint();
    void set_kp(float kp);
    void set_ki(float ki);
    void set_kd(float kd);
    void set_db(float db);
    void set_mo(float mo); 
    float get_kp();
    float get_ki();
    float get_kd();
    float get_db();
    float get_mo();

  private:
    float _kp, _ki, _kd, _db, _mo;
    float _setpoint;
    float _accumulative_error;
    float _difference_error;
    float _rate_error;
    float _controller_output;

};

#endif