/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

int main() {
    stdio_init_all();
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    adc_gpio_init(27);

    while (1) {
        adc_select_input(0);
        uint adc_x_raw = adc_read();
        adc_select_input(1);
        uint adc_y_raw = adc_read();
        // printf("x: %d\n", adc_x_raw);
        // printf("y: %d\n", adc_y_raw);
        // Display the joystick position something like this:
        // X: [            o             ]  Y: [              o         ]
        // const uint bar_width = 40;
        // const uint adc_max = (1 << 12) - 1;
        // uint bar_x_pos = adc_x_raw * bar_width / adc_max;
        // uint bar_y_pos = adc_y_raw * bar_width / adc_max;
        // printf("\rX: [");
        // for (int i = 0; i < bar_width; ++i)
        //     putchar( i == bar_x_pos ? 'o' : ' ');
        // printf("]  Y: [");
        // for (int i = 0; i < bar_width; ++i)
        //     putchar( i == bar_y_pos ? 'o' : ' ');
        // printf("]");
        // front
        if (adc_y_raw > 2500){
            if(adc_x_raw <1700) { // front left
                printf("3 section\n");
            }
            else if(adc_x_raw >3000){ // front right
                printf("1 section\n");
            }
            else {
                printf("2 section\n"); // front
            }
        }
        // back
        else if (adc_y_raw < 2000) {
            if(adc_x_raw <2000) { // back left
                printf("4 section\n");
            }
            else if(adc_x_raw >2000){ // back right
                printf("0 section\n");
            }
        }
        else {
            printf("2 section\n"); // default
        }
    
        sleep_ms(1000);

        // x: [20, 4095]
        // y: [20, 4095]
        // front diagonal: left (1300, 20), right(4095, 4095)
        // back diagonal: left (20, 20), right(4095, 20)

        
    }
}