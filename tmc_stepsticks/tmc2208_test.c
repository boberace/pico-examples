/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"

#include "tmc2208_pico.h"
#include "tmc/helpers/Macros.h"
#include "tmc/ic/TMC2208/TMC2208_Fields.h"


int main() {

    stdio_init_all();
    sleep_ms(2000);

    tmc2208_pico_init();   

    tmc2208_write_register(TMC2208_GCONF, 0b0111000010);


    
    for(int i=0; i < 1500000 ; i=i+1000){
        tmc2208_set_VACTUAL(i);
        sleep_ms(2);
    }
 
 
    int counter = 0;

    while (1) {

        printf("\033[A\33[2K\rtmc2208 test, %i\n", counter);
        sleep_ms(1000);

        counter++;
    }

}



