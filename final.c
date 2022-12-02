/**
 *  V. Hunter Adams (vha3@cornell.edu)
 
    This is an experiment with the multicore capabilities on the
    RP2040. The program instantiates a timer interrupt on each core.
    Each of these timer interrupts writes to a separate channel
    of the SPI DAC and does DDS of two sine waves of two different
    frequencies. These sine waves are amplitude-modulated to "beeps."
    No spinlock is required to mediate the SPI writes because of the
    SPI buffer on the RP2040. Spinlocks are used in the main program
    running on each core to lock the other out from an incrementing
    global variable. These are "under the hood" of the PT_SEM_SAFE_x
    macros. Two threads ping-pong using these semaphores.
    Note that globals are visible from both cores. Note also that GPIO
    pin mappings performed on core 0 can be utilized from core 1.
    Creation of an alarm pool is required to force a timer interrupt to
    take place on core 1 rather than core 0.
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

//Direct Digital Synthesis (DDS) parameters
#define two32 4294967296.0  // 2^32 (a constant)
#define Fs 200000            // sample rate

// the DDS units - core 1
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_1;                  
volatile unsigned int phase_incr_main_1 = (2300.0*two32)/Fs ;

// the DDS units - core 2
// Phase accumulator and phase increment. Increment sets output frequency.
volatile unsigned int phase_accum_main_0;
volatile unsigned int phase_incr_main_0 = (2300.0*two32)/Fs ;

// DDS sine table (populated in main())
#define sine_table_size 256
fix15 sin_table[sine_table_size] ;

// Amplitude modulation parameters and variables
fix15 global_max_amplitude = float2fix15(1.0) ;
fix15 max_amplitude = float2fix15(1.0) ;    // maximum amplitude
fix15 attack_inc ;                          // rate at which sound ramps up
fix15 decay_inc ;                           // rate at which sound ramps down
fix15 current_amplitude_0 = 0 ;             // current amplitude (modified in ISR)
fix15 current_amplitude_1 = 0 ;             // current amplitude (modified in ISR)

// Timing parameters for beeps (units of interrupts)
#define ATTACK_TIME             200
#define DECAY_TIME              200
#define SUSTAIN_TIME            10000
#define BEEP_DURATION           3000
#define BEEP_REPEAT_INTERVAL    40000

// State machine variables
volatile unsigned int STATE_0 = 0 ;
volatile unsigned int count_0 = 0 ;
volatile unsigned int STATE_1 = 2 ;
volatile unsigned int count_1 = 0 ;
volatile unsigned int syl_num_0 = 0 ;
volatile unsigned int syl_num_1 = 0 ;

// SPI data
uint16_t DAC_data_1 ; // output value
uint16_t DAC_data_0 ; // output value

float history_r[17] ;
float history_l[17] ;
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

// Global counter for spinlock experimenting
volatile int global_counter = 0 ;
volatile int paused_0 = 0;
volatile int paused_1 = 0;
// Semaphore
struct pt_sem core_1_go, core_0_go ;

/* Final Project Variable */
#define head_radius 9.0       // a
#define speed_sound 34000.0   // c
int angle_deg;              // user input
fix15 angle_rad; 
int direction_0 = 2;
int direction_1 = 2;
volatile int old_direction_0 = 10;
volatile int old_direction_1 = 10;

float ILD = 0.7;
int ITD = 16; // 45 deg

fix15 max_amplitude_right ;
fix15 attack_inc_right ;
fix15 decay_inc_right ;

fix15 max_amplitude_left ;
fix15 attack_inc_left ;
fix15 decay_inc_left ;

volatile int x_dist = 20 ;
volatile int y_dist = 20 ;

float intensity_diff;

volatile int delay_counter ;

// ADC Channel and pin
#define ADC_CHAN_0 0
#define ADC_CHAN_1 1
#define ADC_CHAN_2 2
#define ADC_PIN_28 28
#define ADC_PIN_26 26
#define ADC_PIN_27 27

// joystick pins
#define JOYSTICK_VY 16
#define JOYSTICK_VX 17

// audio inputs
int adc_audio_0;
int adc_audio_1;

int joystick[4];
int direction;

// This timer ISR is called on core 1 - LEFT
bool repeating_timer_callback_core_1(struct repeating_timer *t) { 

    // second input
    // adc_select_input(0);

    // new_l = adc_read();
    // for (int i =1; i<=16; i++) {
    //     history_l[i] = history_l[i-1];
    // }
    // history_l[0] = new_l;

    // adc_audio_1 = (int) (history_r[ITD]*ILD);

    // DAC_data_1 = (DAC_config_chan_A | (adc_audio_1 & 0xfff))  ;
    
    // spi_write16_blocking(SPI_PORT, &(DAC_data_1), 1) ;   

    if (direction_1 != old_direction_1) {

        if (direction_1==0 || direction_1==4){
            ILD = 0.7 ;
            ITD = 16 ;
        }
        else if (direction_1==1 || direction_1==3){
            ILD = 0.9 ; // 20 degrees
            ITD = 7 ; // 20 degrees
        }
        else if (direction_1==2){
            ILD = 1 ;
            ITD = 0 ;
        }
        old_direction_1 = direction_1;
    }

    if(direction_1==1 || direction_1==0) {

        adc_audio_1 = (int) (history_r[ITD]*ILD);

        DAC_data_1 = (DAC_config_chan_A | (adc_audio_1 & 0xfff))  ;
        
        spi_write16_blocking(SPI_PORT, &(DAC_data_1), 1) ;
    }
    else {

        adc_audio_1 = (int) (history_r[0]);
        // Mask with DAC control bits
        DAC_data_1 = (DAC_config_chan_A | (adc_audio_1 & 0xfff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_1, 1) ;
    }

    return true;
    
}

// This timer ISR is called on core 0 - RIGHT
bool repeating_timer_callback_core_0(struct repeating_timer *t) {

    adc_select_input(2);

    new_r = adc_read();
    for (int i =1; i<=16; i++) {
        history_r[i] = history_r[i-1];
    }
    history_r[0] = new_r;

    // adc_audio_0 = (int) (history_r[0]);

    // DAC_data_0 = (DAC_config_chan_B | (adc_audio_0 & 0xfff))  ;

    // spi_write16_blocking(SPI_PORT, &(DAC_data_0), 1) ;

    // // FOR TWO SOUNDS TO ONE OUTPUT (need to scale values to under 3.3V)
    // adc_select_input(0);
    
    // adc_audio_1 = (int) adc_read();
    
    // // Mask with DAC control bits
    // DAC_data_1 = (DAC_config_chan_A | ((adc_audio_1 + adc_audio_0) & 0xfff))  ;

    // // SPI write (no spinlock b/c of SPI buffer)
    // spi_write16_blocking(SPI_PORT, &DAC_data_1, 1) ;

    // SPI write (no spinlock b/c of SPI buffer)
    // spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

    if (direction_0 != old_direction_0) {

        if (direction_0==0 || direction_0==4){
            ILD = 0.7 ;
            ITD = 16 ;
        }
        else if (direction_0==1 || direction_0==3){
            ILD = 0.9 ; // 20 degrees
            ITD = 7 ; // 20 degrees
        }
        else if (direction_0==2){
            ILD = 1 ;
            ITD = 0 ;
        }
        old_direction_0 = direction_0;
    }

    if(direction_0==3 || direction_0==4){

        adc_audio_0 = (int) (history_r[ITD]*ILD);

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (adc_audio_0 & 0xfff))  ;

        spi_write16_blocking(SPI_PORT, &(DAC_data_0), 1) ;
    }
    else {

        adc_audio_0 = (int) (history_r[0]);

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (adc_audio_0 & 0xfff))  ;

        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;
    }

    return true;
    
}

// This thread runs on core 1
static PT_THREAD (protothread_core_1(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    while(1) {
        // Wait for signal
        PT_SEM_SAFE_WAIT(pt, &core_1_go) ;
        // Turn off LED
        gpio_put(LED, 0) ;
        // Increment global counter variable
        for (int i=0; i<10; i++) {
            global_counter += 1 ;
            sleep_ms(250) ;
        }
        // printf("\n\n") ;
        // signal other core
        PT_SEM_SAFE_SIGNAL(pt, &core_0_go) ;
        printf("Max amplitude left: %lf", fix2float15(max_amplitude_left));
    }
    // Indicate thread end
    PT_END(pt) ;
}

// This thread runs on core 0 - RIGHT ear
static PT_THREAD (protothread_core_0(struct pt *pt))
{
    // Indicate thread beginning
    PT_BEGIN(pt) ;
    while(1) {
        // Wait for signal
        PT_SEM_SAFE_WAIT(pt, &core_0_go) ;
        // Turn on LED
        gpio_put(LED, 1) ;
        // Increment global counter variable
        for (int i=0; i<10; i++) {
            global_counter += 1 ;
            sleep_ms(250) ;
        }
        // printf("\n\n") ;
        // signal other core
        PT_SEM_SAFE_SIGNAL(pt, &core_1_go) ;

        // printf("old directions: %d, %d\n", old_direction_0, old_direction_1);
        // printf("state : %d, %d\n", STATE_0, STATE_1);
        printf("Max amplitude right: %lf", fix2float15(max_amplitude_right));
    }
    // Indicate thread end
    PT_END(pt) ;
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static int test_in ;
    static int old_test = 10;
    static float float_in ;
    while(1) {

        sprintf(pt_serial_out_buffer, "input desired direction (0-5) : ");
        serial_write ;
        serial_read ;
        sscanf(pt_serial_in_buffer,"%d", &test_in) ;
        direction_0 = test_in ;
        direction_1 = test_in ;
    }
    PT_END(pt) ;
}

// User joystick. User can change direction of sound
static PT_THREAD (protothread_joystick(struct pt *pt))
{
    PT_BEGIN(pt) ;

    while(1) {
        adc_select_input(1);
        uint adc_x_raw = adc_read();

        // printf("X raw: %lf", (float) adc_x_raw);

        if(adc_x_raw <2000 && adc_x_raw >50) { // left
            direction = 3;
        }
        else if(adc_x_raw <=50){ // most left
            direction = 4;
        }
        else if(adc_x_raw >=4000){ // most right
            direction = 0;
        }
        else if(adc_x_raw >2500 && adc_x_raw <4000){ // right
            direction = 1;
        }
        else { // front
            direction = 2;
        }

        for (int i =1; i<4; i++) {
            joystick[i] = joystick[i-1];
        }
        joystick[0] = direction;
        if (joystick[0]==joystick[1] && joystick[1]==joystick[2] && joystick[2]==joystick[3]) {
            direction_0 = direction;
            direction_1 = direction;
        }
        
        printf("Direction_0/1: %i\n", direction_0);
        
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
    pt_add_thread(protothread_core_1) ;

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

    // set up increments for calculating bow envelope
    attack_inc = divfix(max_amplitude, int2fix15(ATTACK_TIME)) ;
    decay_inc =  divfix(max_amplitude, int2fix15(DECAY_TIME)) ;

    // Build the sine lookup table
    // scaled to produce values between 0 and 4096 (for 12-bit DAC)
    int ii;
    for (ii = 0; ii < sine_table_size; ii++){
         sin_table[ii] = float2fix15(2047*sin((float)ii*6.283/(float)sine_table_size));
    }

    // Initialize the intercore semaphores
    PT_SEM_SAFE_INIT(&core_0_go, 1) ;
    PT_SEM_SAFE_INIT(&core_1_go, 0) ;

    // Desynchronize the beeps
    // sleep_ms(fix2int15(ITD)*1000) ;
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
    pt_add_thread(protothread_core_0) ;

    // add user interface
    // pt_add_thread(protothread_serial) ;

    // add joystick interface
    pt_add_thread(protothread_joystick) ;

    // Start scheduling core 0 threads
    pt_schedule_start ;

}