/*
Minjung Kwon, Ann Xu, Amy Wang
Spacial Audio Project

This project implements spatial audio into a fun and interactive murder mystery game. 
We generated self-recorded audio and passed it to headphones while applying a spatial audio algorithm to the input audio. 
To the headphone user, the audio appears to be coming from the specific direction the user chooses with the joystick. 
The goal is to closely mimic how sounds in real life reach our ears, depending on where the sound is coming from.
 */

// Include necessary libraries
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/sync.h"
#include "hardware/spi.h"
#include "hardware/adc.h"

// Include protothreads
#include "pt_cornell_rp2040_v1.h"

// Macros for fixed-point arithmetic (faster than floating point)
typedef signed int fix15 ;
#define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
#define float2fix15(a) ((fix15)((a)*32768.0)) 
#define fix2float15(a) ((float)(a)/32768.0)
#define absfix15(a) abs(a) 
#define int2fix15(a) ((fix15)(a << 15))
#define fix2int15(a) ((int)(a >> 15))
#define char2fix15(a) (fix15)(((fix15)(a)) << 15)
#define divfix(a,b) (fix15)( (((signed long long)(a)) << 15) / (b))

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

float history_r[21] ;
float history_l[21] ;
float new_r ;
float new_l ;

// DAC parameters (see the DAC datasheet)
// A-channel, 1x, active
#define DAC_config_chan_A 0b0011000000000000
// B-channel, 1x, active
#define DAC_config_chan_B 0b1011000000000000

//SPI configurations (note these represent GPIO number, NOT pin number)
#define PIN_MISO 4
#define PIN_CS   5
#define PIN_SCK  6
#define PIN_MOSI 7
#define LDAC     8
#define LED      25
#define SPI_PORT spi0
#define CORE_0   2
#define CORE_1   3

// Semaphore
struct pt_sem core_1_go, core_0_go ;

/* Final Project Variable */
#define head_radius 9.0       // a
#define speed_sound 34000.0   // c
int direction_r0 = 1;
int direction_r1 = 1;
volatile int old_direction_r0 = 10;
volatile int old_direction_r1 = 10;

int direction_l0 = 3;
int direction_l1 = 3;
volatile int old_direction_l0 = 10;
volatile int old_direction_l1 = 10;

float ILD_r;
int ITD_r;
float ILD_l;
int ITD_l; 

// ADC Channel and pin
#define ADC_CHAN_0 0
#define ADC_CHAN_1 1
#define ADC_CHAN_2 2
#define ADC_PIN_28 28
#define ADC_PIN_26 26
#define ADC_PIN_27 27

// audio inputs
int adc_audio_r0;
int adc_audio_r1;
int adc_audio_l0;
int adc_audio_l1;

int joystick[4];
int direction = 2;

// This timer ISR is called on core 1 - LEFT
bool repeating_timer_callback_core_1(struct repeating_timer *t) { 

    // second input
    adc_select_input(0);

    new_l = adc_read();
    for (int i =1; i<=20; i++) {
        history_l[i] = history_l[i-1];
    }
    history_l[0] = new_l;

    if (direction_r1 != old_direction_r1) {

        if (direction_r1==0 || direction_r1==4){
            ILD_r = 0.5 ; // 80 degrees
            ITD_r = 20 ; // 80 degrees
        }
        else if (direction_r1==1 || direction_r1==3){
            ILD_r = 0.7 ; // 45 degrees
            ITD_r = 16 ; // 45 degrees
        }
        else if (direction_r1==2){
            ILD_r = 1 ; // 0 degrees
            ITD_r = 0 ; // 0 degrees
        }
        old_direction_r1 = direction_r1;
    }

    if (direction_l1 != old_direction_l1) {

        if (direction_l1==0 || direction_l1==4){
            ILD_l = 0.5 ; // 80 degrees
            ITD_l = 20 ; // 80 degrees
        }
        else if (direction_l1==1 || direction_l1==3){
            ILD_l = 0.7 ; // 45 degrees
            ITD_l = 16 ; // 45 degrees
        }
        else if (direction_l1==2){
            ILD_l = 1 ; // 0 degrees
            ITD_l = 0 ; // 0 degrees
        }
        old_direction_l1 = direction_l1;
    }

    if (direction_r1==1) {
        adc_audio_r1 = (int) (history_r[ITD_r]*ILD_r)/2;
    }
    else if (direction_r1==0) {
        adc_audio_r1 = (int) (history_r[ITD_r]*ILD_r)/10;
    }
    else if (direction_r1==4) {
        adc_audio_r1 = (int) (history_r[0])/10;
    }
    else { // direction 1,2
        adc_audio_r1 = (int) (history_r[0])/2;
    }

    if (direction_l1==1) {
        adc_audio_l1 = (int) (history_l[ITD_l]*ILD_l)/2;
    }
    else if (direction_l1==0) {
        adc_audio_l1 = (int) (history_l[ITD_l]*ILD_l)/10;
    }
    else if (direction_l1==4){
        adc_audio_l1 = (int) (history_l[0])/10;
    }
    else { // direction 1,2
        adc_audio_l1 = (int) (history_l[0])/2;
    }

    DAC_data_1 = (DAC_config_chan_A | ((adc_audio_r1 + adc_audio_l1) & 0xfff))  ;
    spi_write16_blocking(SPI_PORT, &DAC_data_1, 1) ;

    return true;
    
}

// This timer ISR is called on core 0 - RIGHT
bool repeating_timer_callback_core_0(struct repeating_timer *t) {

    adc_select_input(2);

    new_r = adc_read();
    for (int i =1; i<=20; i++) {
        history_r[i] = history_r[i-1];
    }
    history_r[0] = new_r;

    if (direction_r0 != old_direction_r0) {

        if (direction_r0==0 || direction_r0==4){
            ILD_r = 0.5 ; // 80 degrees
            ITD_r = 20 ; // 80 degrees
        }
        else if (direction_r0==1 || direction_r0==3){
            ILD_r = 0.7 ; // 45 degrees
            ITD_r = 16 ; // 45 degrees
        }
        else if (direction_r0==2){
            ILD_r = 1 ; // 0 degrees
            ITD_r = 0 ; // 0 degrees
        }
        old_direction_r0 = direction_r0;
    }

    if (direction_l0 != old_direction_l0) {

        if (direction_l0==0 || direction_l0==4){
            ILD_l = 0.5 ; // 80 degrees
            ITD_l = 20 ; // 80 degrees
        }
        else if (direction_l0==1 || direction_l0==3){
            ILD_l = 0.7 ; // 45 degrees
            ITD_l = 16 ; // 45 degrees
        }
        else if (direction_l0==2){
            ILD_l = 1 ; // 0 degrees
            ITD_l = 0 ; // 0 degrees
        }
        old_direction_l0 = direction_l0;
    }

    if (direction_r0==3){
        adc_audio_r0 = (int) (history_r[ITD_r]*ILD_r)/2;
    }
    else if (direction_r0==4){
        adc_audio_r0 = (int) (history_r[ITD_r]*ILD_r)/10;
    }
    else if (direction_r0==0){
        adc_audio_r0 = (int) (history_r[0]/10);
    }
    else { // direction 1,2
        adc_audio_r0 = (int) (history_r[0]/2);
    }
    
    if (direction_l0==3) {
        adc_audio_l0 = (int) (history_l[ITD_l]*ILD_l)/2;
    }
    else if (direction_l0==4) {
        adc_audio_l0 = (int) (history_l[ITD_l]*ILD_l)/10;
    }
    else if (direction_l0==0){
        adc_audio_l0 = (int) (history_l[0])/10;
    }
    else { // direction 1,2
        adc_audio_l0 = (int) (history_l[0])/2;
    }

    DAC_data_0 = (DAC_config_chan_B | ( adc_audio_r0 + adc_audio_l0 & 0xfff))  ;
    spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

    return true;
    
}

// User joystick. User can change direction of sound
static PT_THREAD (protothread_joystick(struct pt *pt))
{
    PT_BEGIN(pt) ;

    while(1) {
        PT_YIELD_usec(40000);
        adc_select_input(1);
        uint adc_x_raw = adc_read();

        // if(adc_x_raw <1500 && adc_x_raw >100) { // left
        //     direction = 3;
        // }
        // else if(adc_x_raw <= 100){ // most left
        //     direction = 4;
        // }
        // else if(adc_x_raw >=4000){ // most right
        //     direction = 0;
        // }
        // else if(adc_x_raw >3000 && adc_x_raw <4000){ // right
        //     direction = 1;
        // }
        // else { // front
        //     direction = 2;
        // }

        if(adc_x_raw <1000) {
            direction=4;
        }
        else if (adc_x_raw >3000) {
            direction=0;
        }

        for (int i =1; i<4; i++) {
            joystick[i] = joystick[i-1];
        }
        joystick[0] = direction;
        if (joystick[0]==joystick[1] && joystick[1]==joystick[2] && joystick[2]==joystick[3]) {
            if (direction==0 || direction==1) {
                direction_r0 = 2;
                direction_r1 = 2;
                direction_l0 = 4;
                direction_l1 = 4;
            }
            else if (direction==3 || direction==4) {
                direction_r0 = 0;
                direction_r1 = 0;
                direction_l0 = 2;
                direction_l1 = 2;
            }
        }
        
    }
    PT_END(pt) ;
}

// This is the core 1 entry point - LEFT ear
void core1_entry() {
    // create an alarm pool on core 1
    alarm_pool_t *core1pool ;
    core1pool = alarm_pool_create(2, 16) ;

    // Create a repeating timer that calls repeating_timer_callback.
    struct repeating_timer timer_core_1;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
    alarm_pool_add_repeating_timer_us(core1pool, -25, 
        repeating_timer_callback_core_1, NULL, &timer_core_1);

    // Add thread to core 1
    // pt_add_thread(protothread_core_1) ;

    // Start scheduler on core 1
    pt_schedule_start ;

}


// Core 0 entry point
int main() {
    // Initialize stdio/uart (printf won't work unless you do this!)
    stdio_init_all();
    printf("Hello, friends!\n");

    // Initialize SPI channel (channel, baud rate set to 20MHz)
    spi_init(SPI_PORT, 20000000) ;
    // Format (channel, data bits per transfer, polarity, phase, order)
    spi_set_format(SPI_PORT, 16, 0, 0, 0);

    // Map SPI signals to GPIO ports
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI) ;
    gpio_set_function(CORE_0, GPIO_FUNC_SPI);
    gpio_set_function(CORE_1, GPIO_FUNC_SPI) ;

    // Map LDAC pin to GPIO port, hold it low (could alternatively tie to GND)
    gpio_init(LDAC) ;
    gpio_set_dir(LDAC, GPIO_OUT) ;
    gpio_put(LDAC, 0) ;

    // Map LED to GPIO port, make it low
    gpio_init(LED) ;
    gpio_set_dir(LED, GPIO_OUT) ;
    gpio_put(LED, 0) ;

    gpio_init(CORE_0) ;
    gpio_init(CORE_1) ;

    // ADC INIT FOR JOYSTICK
    stdio_init_all();
    adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);

    // Initialize the intercore semaphores
    PT_SEM_SAFE_INIT(&core_0_go, 1) ;
    PT_SEM_SAFE_INIT(&core_1_go, 0) ;

    // Launch core 1
    multicore_launch_core1(core1_entry);

    // Create a repeating timer that calls 
    // repeating_timer_callback (defaults core 0)

    struct repeating_timer timer_core_0;

    // Negative delay so means we will call repeating_timer_callback, and call it
    // again 25us (40kHz) later regardless of how long the callback took to execute
   
    add_repeating_timer_us(-25, 
        repeating_timer_callback_core_0, NULL, &timer_core_0);

    // Add core 0 threads
    // pt_add_thread(protothread_core_0) ;

    // add joystick interface
    pt_add_thread(protothread_joystick) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}
