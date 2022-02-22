#ifndef bmc_h
#define bmc_h

#include <stdint.h> 
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "math.h"

class bmc {
  public:
    bmc(uint8_t pinAB, bool slowdecay = false);
    void run(float duty); 

  private:
    bool _slowdecay;
    uint8_t _pwm_slice;
    uint16_t _pwm_TOP;
    bool _pwm_on;
    uint8_t _pinA;
    uint8_t _pinB; 
};

#endif
