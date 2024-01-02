#ifndef TMC2208_PICO_H
#define TMC2208_PICO_H

#include "tmc/ic/TMC2208/TMC2208.h"


static uint8_t reset();
static uint8_t restore();
void tmc2208_pico_init();
void tmc2208_write_register(uint8_t address, uint32_t value);
void tmc2208_read_register(uint8_t address, uint32_t *value);

void tmc2208_set_VACTUAL(int32_t value);



#endif //TMC2208_PICO_H



