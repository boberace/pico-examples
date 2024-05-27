#include "bmc.h"

bmc::bmc( uint8_t pinAB, bool slowdecay) {
  _pinA = pinAB; 
  _pinB = _pinA + 1; 
  _slowdecay = slowdecay;
  gpio_set_function(_pinA, GPIO_FUNC_PWM);
  _pwm_slice = pwm_gpio_to_slice_num(_pinA);
  gpio_set_function(_pinB, GPIO_FUNC_PWM);

  // uint32_t fsys = clock_get_hz(clk_sys);
  // 4.5.2.6. Configuring PWM Period
  _pwm_TOP = 24999; 
  float pwm_DIV = 200.0;
  bool pwm_CSR = 0;
  _pwm_on = false;
  // uint32_t pwm_freq = fsys / ((_pwm_TOP + 1)*(pwm_CSR+1)*(pwm_DIV));    
  pwm_config config = pwm_get_default_config();
  pwm_config_set_wrap(&config, _pwm_TOP); 
  pwm_config_set_clkdiv(&config, pwm_DIV);
  pwm_config_set_phase_correct(&config,pwm_CSR);
  pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);
  pwm_config_set_output_polarity(&config, 0, 0);
  pwm_init(_pwm_slice, &config, 0);
}

void bmc::run(float duty) {
  if(duty <= 1 && duty >= -1){   
    if(duty != 0.0){

      if(_pwm_on == false) {
        gpio_set_function(_pinA, GPIO_FUNC_PWM);
        gpio_set_function(_pinB, GPIO_FUNC_PWM);
        pwm_set_enabled(_pwm_slice, _pwm_on = true);      
      }

      if(_slowdecay){
        uint16_t level = uint16_t(abs(duty)*_pwm_TOP);
        if (duty > 0) pwm_set_both_levels(_pwm_slice,level,0);
        else          pwm_set_both_levels(_pwm_slice,0,level);
      } else {
        uint16_t level = _pwm_TOP - uint16_t(abs(duty)*_pwm_TOP);
        if (duty > 0) pwm_set_both_levels(_pwm_slice,_pwm_TOP,level);             
        else          pwm_set_both_levels(_pwm_slice,level,_pwm_TOP);
      } 
    } else {        
        pwm_set_both_levels(_pwm_slice,0,0);
        gpio_set_outover(_pinA, GPIO_OVERRIDE_LOW );
        gpio_set_outover(_pinB, GPIO_OVERRIDE_LOW );
        pwm_set_enabled(_pwm_slice, _pwm_on = false);
    } 

  } else {
    // todo - throw an out of range flag
        pwm_set_both_levels(_pwm_slice,0,0);
        gpio_set_outover(_pinA, GPIO_OVERRIDE_LOW );
        gpio_set_outover(_pinB, GPIO_OVERRIDE_LOW );
        pwm_set_enabled(_pwm_slice, _pwm_on = false);
  }
} 

