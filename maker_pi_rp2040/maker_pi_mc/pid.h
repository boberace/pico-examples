#ifndef pid_h
#define pid_h
// creates a simple pid instance
// the setpoint is between -1 to 1
// the sensor input is between -1 to 1

class pid {
  public:
    pid(float kp = 0.25, float ki = 0.1, float kd=0.01);
    float output_update(float sensor_input, float time_change); 
    void set_setpoint(float sp);
    float get_setpoint();
    void set_kp(float kp);
    void set_ki(float ki);
    void set_kd(float kd);
    float get_kp();
    float get_ki();
    float get_kd();

  private:
    float _kp, _ki, _kd;
    float _setpoint;
    float _accumulative_error;
    float _previous_difference_error;
    float _controller_output;
};

#endif