#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/timer.h"

#include "quadrature_encoder.pio.h"

#define PIN_CA1 18
#define PIN_CA2 19

#define PIN_HALL_A 2
#define PIN_HALL_B 3

#define PIN_LED 15

bool clockwise = true;

PIO pio_QE = pio0;
uint sm_QE_MA = 0;

#define UART_A_ID uart1
#define UART_A_BAUD_RATE 115200
#define UART_A_TX_PIN 16
#define UART_A_RX_PIN 17

uint setup_uart();
void setup_motor();

static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint pin, int max_step_rate);
static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm);

int main() {
    int new_value_M1, delta_M1, old_value_M1 = 0;
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    stdio_init_all();
    sleep_ms(1000);
    setup_uart();
    setup_motor();

    pio_add_program(pio_QE, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio_QE, sm_QE_MA, PIN_HALL_A, 0);



    while (1)
    {
        gpio_put(PIN_LED, !gpio_get(PIN_LED));

        new_value_M1 = quadrature_encoder_get_count(pio_QE, sm_QE_MA);
        delta_M1 = new_value_M1 - old_value_M1;
        old_value_M1 = new_value_M1;

        if(new_value_M1 < 600)
        {
            gpio_put(PIN_CA1, 0);
            gpio_put(PIN_CA2, 1);
        }
        else
        {
            gpio_put(PIN_CA1, 0);
            gpio_put(PIN_CA2, 0);
        }

        if(delta_M1)
        printf("\033[A\33[2K\rposition %8d, delta %6d\n", new_value_M1, delta_M1);
   

    }

}

uint setup_uart() {
    uint uart_ret = uart_init(UART_A_ID, UART_A_BAUD_RATE);
    gpio_set_function(UART_A_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_A_RX_PIN, GPIO_FUNC_UART);
    return uart_ret;
}

void setup_motor() {
    gpio_init(PIN_CA1);
    gpio_set_dir(PIN_CA1, GPIO_OUT);
    gpio_put(PIN_CA1, 0);

    gpio_init(PIN_CA2);
    gpio_set_dir(PIN_CA2, GPIO_OUT);
    gpio_put(PIN_CA2, 0);

    gpio_init(PIN_HALL_A);
    gpio_set_dir(PIN_HALL_A, GPIO_IN);

    gpio_init(PIN_HALL_B);
    gpio_set_dir(PIN_HALL_B, GPIO_IN);
}

// max_step_rate is used to lower the clock of the state machine to save power
// if the application doesn't require a very high sampling rate. Passing zero
// will set the clock to the maximum

static inline void quadrature_encoder_program_init(PIO pio, uint sm, uint pin, int max_step_rate)
{
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 2, false);
    gpio_pull_up(pin);
    gpio_pull_up(pin + 1);

    pio_sm_config c = quadrature_encoder_program_get_default_config(0);

    sm_config_set_in_pins(&c, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&c, pin); // for JMP
    // shift to left, autopull disabled
    sm_config_set_in_shift(&c, false, false, 32);
    // don't join FIFO's
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_NONE);

    // passing "0" as the sample frequency,
    if (max_step_rate == 0) {
        sm_config_set_clkdiv(&c, 1.0);
    } else {
        // one state machine loop takes at most 10 cycles
        float div = (float)clock_get_hz(clk_sys) / (10 * max_step_rate);
        sm_config_set_clkdiv(&c, div);
    }

    pio_sm_init(pio, sm, 0, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline int32_t quadrature_encoder_get_count(PIO pio, uint sm)
{
    uint ret;
    int n;

    // if the FIFO has N entries, we fetch them to drain the FIFO,
    // plus one entry which will be guaranteed to not be stale
    n = pio_sm_get_rx_fifo_level(pio, sm) + 1;
    while (n > 0) {
        ret = pio_sm_get_blocking(pio, sm);
        n--;
    }
    return ret;
}
