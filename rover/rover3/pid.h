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

    /*! \brief Constructor for the PID class
     *  \param kp Proportional gain
     *  \param ki Integral gain
     *  \param kd Derivative gain
     *  \param db Deadband
     *  \param mo Max output
    */
    pid(float kp = 0.25, float ki = 0.1, float kd=0.01);
    
    /*! \brief Update the PID controller
     *  \param sensor_input The current value in reference to the setpoint
     *  \param time_change The time change
     *  \return The controller output
    */
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
    float _kp, _ki, _kd;
    float _setpoint;
    float _accumulative_error;
    float _difference_error;
    float _rate_error;
    float _controller_output;

};

#endif