#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <string.h> 
#include <stdio.h>

static const uint8_t M24C64_ADDR = 0x50; // Default I2C address for M24C64
static const uint8_t M24C64_SIZE = 8192; // 64Kbit = 8192 bytes
static const uint8_t M24C64_PAGE_SIZE = 32; // 32 bytes per page

static const uint8_t I2CA_SDA_PIN = 26;
static const uint8_t I2CA_SCL_PIN = 27;
static i2c_inst_t *I2CA_INST = i2c1;

// Initialize I2CA_INST
void I2CA_INST_init(i2c_inst_t *i2c, uint sda_pin, uint scl_pin) {
    i2c_init(i2c, 400 * 1000);
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
}

// Write data to EEPROM
int m24c64_write(i2c_inst_t *i2c, uint16_t addr, const uint8_t *data, size_t length) {
    uint8_t buffer[length + 2];
    buffer[0] = (addr >> 8) & 0xFF; // Address high byte
    buffer[1] = addr & 0xFF;       // Address low byte
    memcpy(buffer + 2, data, length);
    return i2c_write_blocking(i2c, M24C64_ADDR, buffer, length + 2, false);
}

// Read data from EEPROM
int m24c64_read(i2c_inst_t *i2c, uint16_t addr, uint8_t *data, size_t length) {
    uint8_t buffer[2];
    buffer[0] = (addr >> 8) & 0xFF; // Address high byte
    buffer[1] = addr & 0xFF;       // Address low byte
    i2c_write_blocking(i2c, M24C64_ADDR, buffer, 2, true); // Send address
    return i2c_read_blocking(i2c, M24C64_ADDR, data, length, false);
}

int main() {
    stdio_init_all();
    I2CA_INST_init(I2CA_INST, I2CA_SDA_PIN, I2CA_SCL_PIN); 
    sleep_ms(2000);

    uint8_t write_data[] = {'H', 'e', 'l', 'l', 'o', ' ', 'E', 'E', 'P', 'R', 'O', 'M'};
    m24c64_write(I2CA_INST, 0x0000, write_data, sizeof(write_data));

    sleep_ms(10); // Small delay for EEPROM write

    uint8_t read_data[12];
    m24c64_read(I2CA_INST, 0x0000, read_data, sizeof(read_data));

    printf("Read Data: %s\n", read_data);

    uint counter = 0;
    while (true) {
        printf("Hello, eeprom! %d\n", counter);
        sleep_ms(1000);
        counter++;
    }

    return 0;
}