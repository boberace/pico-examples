#include "tmc2208_pico.h"
#include "tmc/ic/TMC2208/TMC2208_Fields.h"
#include "pico/stdlib.h"

#include "tmc/helpers/CRC.h"
#include <stdlib.h>
#include <stdio.h>
#include "hardware/uart.h"

// #define DEBUG_PRINT // uncomment to print debug messages

#define UART_ID uart0
#define BAUD_RATE 115200

#define UART_TX_PIN 16
#define UART_RX_PIN 17


#ifdef DEBUG_PRINT
#define PRINT(...) printf(__VA_ARGS__)
#else
#define PRINT(...)
#endif


TMC2208TypeDef TMC2208;
ConfigurationTypeDef *TMC2208_config;
#define TMC2208_CRC(data, length) tmc_CRC8(data, length, 1)

void tmc2208_pico_init(){

    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    tmc2208_init(&TMC2208, 0, TMC2208_config, &tmc2208_defaultRegisterResetState[0]);

    TMC2208.config->reset = reset;
    TMC2208.config->restore = restore;

	tmc_fillCRC8Table(0x07, true, 1);

}

static uint8_t reset(){

	return tmc2208_reset(&TMC2208);
}

static uint8_t restore()
{
	return tmc2208_restore(&TMC2208);
}

// => UART wrapper
void tmc2208_readWriteArray(uint8_t channel, uint8_t *data, size_t writeLength, size_t readLength)
{
    UNUSED(channel);
    
    // write data to uart then read data from uart
    if (writeLength > 0){
        uart_write_blocking(UART_ID, data, writeLength);
        PRINT("writeLength: %d\n", writeLength);
        for (int i = 0 ; i < writeLength; i++) {
            PRINT("%d\n", (data[i]));
        }
        uint8_t dummy[writeLength];
        uart_read_blocking(UART_ID, dummy, writeLength); // single wire uart - clear out send data
    }

    PRINT("\n");
    if (readLength > 0){
        uart_read_blocking(UART_ID, data, readLength);
        PRINT("readLength: %d\n", readLength);
        for (int i = 0 ; i < readLength; i++) {
            PRINT("%d\n", (data[i]));
        }
    }

}
// <= UART wrapper

// => CRC wrapper
uint8_t tmc2208_CRC8(uint8_t *data, size_t length)
{
	return TMC2208_CRC(data, length);
}
// <= CRC wrapper

void tmc2208_write_register(uint8_t address, uint32_t value)
{
    tmc2208_writeInt(&TMC2208, address, value);
}

void tmc2208_read_register(uint8_t address, uint32_t *value)
{
    *value = tmc2208_readInt(&TMC2208, address);
}

void tmc2208_set_VACTUAL(int32_t value){
    FIELD_WRITE(tmc2208_writeInt, &TMC2208, TMC2208_VACTUAL, TMC2208_VACTUAL_MASK, TMC2208_VACTUAL_SHIFT, value);
}