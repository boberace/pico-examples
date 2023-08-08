#include <stdio.h>
#include "pico/stdlib.h"
#include "cindex.h"

int main() {
    stdio_init_all();

    cindex cdx[8] = {10, 10, 10, 10, 10, 10, 10, 10};

    int counter = 0;
    sleep_ms(3000);

    while(counter < 15){
        printf("a++: %i, ++b: %i, ", static_cast<int>(cdx[0]++), static_cast<int>(++cdx[1]));
        printf("c--: %i, --d: %i, ", static_cast<int>(cdx[2]--), static_cast<int>(--cdx[3]));
        cdx[4]+=3;
        cdx[5]-=3;
        printf("e+=3: %i, f-=3: %i, ", static_cast<int>(cdx[4]), static_cast<int>(cdx[5]));
        cdx[6] = cdx[6] + 3;
        cdx[7] = cdx[7] - 3;
        printf("g=g+3: %i, h=h-3: %i, ", static_cast<int>(cdx[6]), static_cast<int>(cdx[7]));
        printf("\n");
        counter++;
    }  

    printf("\n");
    for (int i = 0; i < 8; ++i) {
        cdx[i].set_top(20);
    }

    while(counter < 30){
        printf("a++: %i, ++b: %i, ", static_cast<int>(cdx[0]++), static_cast<int>(++cdx[1]));
        printf("c--: %i, --d: %i, ", static_cast<int>(cdx[2]--), static_cast<int>(--cdx[3]));
        cdx[4]+=3;
        cdx[5]-=3;
        printf("e+=3: %i, f-=3: %i, ", static_cast<int>(cdx[4]), static_cast<int>(cdx[5]));
        cdx[6] = cdx[6] + 3;
        cdx[7] = cdx[7] - 3;
        printf("g=g+3: %i, h=h-3: %i, ", static_cast<int>(cdx[6]), static_cast<int>(cdx[7]));
        printf("\n");
        counter++;
    }  
}