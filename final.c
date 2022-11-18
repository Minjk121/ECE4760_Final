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

// Values output to DAC
int DAC_output_0 ;
int DAC_output_1 ;

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

// Two variables to store core number
volatile int corenum_0  ;
volatile int corenum_1  ;

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
int direction_0 = 0;
int direction_1 = 0;
volatile int old_direction_0 = 0;
volatile int old_direction_1 = 0;
// fix15 ITD = float2fix15(0.00039507 * 40000); // only for 45 deg : timing delay (seconds)
// fix15 ILD = float2fix15(0.7071);     // level delay - 45 deg
// int ITD = 16; //- 45 deg
// int ITD = 20; // 60 deg
// fix15 ILD = float2fix15(0.5);     // level delay - 60 deg

fix15 ILD = float2fix15(0.7);     // level delay - 45 deg
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
#define ADC_PIN_26 26
#define ADC_PIN_27 27


// This timer ISR is called on core 1 - LEFT
bool repeating_timer_callback_core_1(struct repeating_timer *t) {
    if (direction_1 != old_direction_1) {
        old_direction_1 = direction_1;
        if (direction_1==2) {
            STATE_1=0;
            count_1=0;
            current_amplitude_1 = 0 ;
        }
        else if (direction_1==0 || direction_1==1) {
            STATE_1=2;
            count_1=0;
            current_amplitude_1 = 0 ;
        }
        else if (direction_1==3 || direction_1==4) {
            STATE_1=0;
            count_1=0;
            current_amplitude_1 = 0 ;
        }
    }

    if (direction_1==0 || direction_1==4){
        ILD = float2fix15(0.7) ;
        ITD = 16 ;
    }
    else if (direction_1==1 || direction_1==3){
        ILD = float2fix15(0.9) ; // 20 degrees
        ITD = 7 ; // 20 degrees
    }
    else if (direction_1==2){
        ILD = float2fix15(1) ;
        ITD = 0 ;
    }
        
    if (STATE_1 == 0) {
        // DDS phase and sine table lookup
        phase_accum_main_1 += phase_incr_main_1  ;
        DAC_output_1 = fix2int15(multfix15(current_amplitude_1,
            sin_table[phase_accum_main_1>>24])) + 2048 ;
        
        // intensity decay tuning -- customized for each audio source
        // intensity_diff = (10 * log(sqrt((x_dist*x_dist)+(y_dist*y_dist))/6) / log(10)) - 2;
        // max_amplitude = global_max_amplitude - float2fix15(intensity_diff);

        // if (max_amplitude < 0) max_amplitude = int2fix15(0);
        // else if (max_amplitude > global_max_amplitude) max_amplitude = global_max_amplitude;

        // max_amplitude_left = multfix15( max_amplitude , ILD);


        max_amplitude_left = multfix15( max_amplitude , ILD);
        attack_inc_left = divfix(max_amplitude_left, int2fix15(ATTACK_TIME));
        decay_inc_left = divfix(max_amplitude_left, int2fix15(DECAY_TIME));

        if(direction_1==1 || direction_1==0) {
            // Ramp up amplitude
            if (count_1 < ATTACK_TIME) {
                current_amplitude_1 = (current_amplitude_1 + attack_inc_left) ;
            }
            // Ramp down amplitude
            else if (count_1 > BEEP_DURATION - DECAY_TIME) {
                current_amplitude_1 = (current_amplitude_1 - decay_inc_left) ;
            }
        }
        else {
            // Ramp up amplitude
            if (count_1 < ATTACK_TIME) {
                current_amplitude_1 = (current_amplitude_1 + attack_inc) ;
            }
            // Ramp down amplitude
            else if (count_1 > BEEP_DURATION - DECAY_TIME) {
                current_amplitude_1 = (current_amplitude_1 - decay_inc) ;
            }
        }

        // Mask with DAC control bits
        DAC_data_1 = (DAC_config_chan_A | (DAC_output_1 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_1, 1) ;

        // Increment the counter
        count_1 += 1 ;

        // State transition?
        if (count_1 == BEEP_DURATION) {
            STATE_1 = 1 ;
            count_1 = 0 ;
        }
    }
    else if(STATE_1 == 1) {
        count_1 += 1 ;
        if (count_1 == BEEP_REPEAT_INTERVAL) {
            count_1 = 0 ;
            current_amplitude_1 = 0 ;

            if(direction_1==1 || direction_1==0){
                STATE_1 = 2 ;           
            }else{
                STATE_1 = 0 ;
            }
        }
    }else if(STATE_1 == 2){
        count_1 += 1 ;
        if (count_1 == ITD) {
            count_1 = 0 ;
            current_amplitude_1 = 0 ;
            STATE_1 = 0 ;           
        }
    }
    // retrieve core number of execution
    corenum_1 = get_core_num() ;

    return true;
    
}

// This timer ISR is called on core 0 - RIGHT
bool repeating_timer_callback_core_0(struct repeating_timer *t) {
    // delay_counter++;

    if (direction_0 != old_direction_0) {
        old_direction_0 = direction_0;
        if (direction_0==2) {
            STATE_0=0;
            count_0=0;
            current_amplitude_0 = 0 ;
        }
        else if (direction_0==0 || direction_0==1) {
            STATE_0=0;
            count_0=0;
            current_amplitude_0 = 0 ;
        }
        else if (direction_0==3 || direction_0==4) {
            STATE_0=2;
            count_0=0;
            current_amplitude_0 = 0 ;
        }
    }

    if (direction_0==0 || direction_0==4){
        ILD = float2fix15(0.7) ;
        ITD = 16 ;
    }
    else if (direction_0==1 || direction_0==3){
        ILD = float2fix15(0.9) ; // 20 degrees
        ITD = 7 ; // 20 degrees
    }
    else if (direction_0==2){
        ILD = float2fix15(1) ;
        ITD = 0 ;
    }

    if (STATE_0 == 0) {

        // DDS phase and sine table lookup
        phase_accum_main_0 += phase_incr_main_0  ;
        DAC_output_0 = fix2int15(multfix15(current_amplitude_0,
            sin_table[phase_accum_main_0>>24])) + 2048 ;


        max_amplitude_right = multfix15( max_amplitude , ILD);
        attack_inc_right = divfix(max_amplitude_right, int2fix15(ATTACK_TIME));
        decay_inc_right = divfix(max_amplitude_right, int2fix15(DECAY_TIME));

        if(direction_0==3 || direction_0==4){
            // Ramp up amplitude
            if (count_0 < ATTACK_TIME) {
                current_amplitude_0 = (current_amplitude_0 + attack_inc_right) ;
            }
            // Ramp down amplitude
            else if (count_0 > BEEP_DURATION - DECAY_TIME) {
                current_amplitude_0 = (current_amplitude_0 - decay_inc_right) ;
            }
        }
        else {
            // Ramp up amplitude
            if (count_0 < ATTACK_TIME) {
                current_amplitude_0 = (current_amplitude_0 + attack_inc) ;
            }
            // Ramp down amplitude
            else if (count_0 > BEEP_DURATION - DECAY_TIME) {
                current_amplitude_0 = (current_amplitude_0 - decay_inc) ;
            }
        }

        // Mask with DAC control bits
        DAC_data_0 = (DAC_config_chan_B | (DAC_output_0 & 0xffff))  ;

        // SPI write (no spinlock b/c of SPI buffer)
        spi_write16_blocking(SPI_PORT, &DAC_data_0, 1) ;

        // Increment the counter
        count_0 += 1 ;

        // State transition?
        if (count_0 == BEEP_DURATION) {
            STATE_0 = 1 ;
            count_0 = 0 ;
        }
    }
    else if(STATE_0 == 1) {
        count_0 += 1 ;
        if (count_0 == BEEP_REPEAT_INTERVAL) {
            count_0 = 0 ;
            current_amplitude_0 = 0 ;

            if(direction_0==3 || direction_0==4){
                STATE_0 = 2 ;           
            }else{
                STATE_0 = 0 ;
            }
        }
    }else if(STATE_0 == 2){
        count_0 += 1 ;
        if (count_0 == ITD) {
            count_0 = 0 ;
            current_amplitude_0 = 0 ;
            STATE_0 = 0 ;      
        }
    }
    // retrieve core number of execution
    corenum_0 = get_core_num() ;
    // if (delay_counter == ITD*40000) {
    //     delay_counter = 0;
        
    // }
    

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
            // printf("Core 1: %d, ISR core: %d\n", global_counter, corenum_1) ;
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
            // printf("Core 0: %d, ISR core: %d\n", global_counter, corenum_0) ;
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


        // printf("input desired direction (0-5) : ");
        // if (test_in==2) {
        //     STATE_0=0;
        //     STATE_1=0;
        // }
        // else if (test_in==0 || test_in==1) {
        //     STATE_0=0;
        //     STATE_1=2;
        // }
        // else if (test_in==3 || test_in==4) {
        //     STATE_0=2;
        //     STATE_1=0;
        // }
    }
    PT_END(pt) ;
}

// User joystick. User can change direction of sound
static PT_THREAD (protothread_joystick(struct pt *pt))
{
    PT_BEGIN(pt) ;

    while(1) {
        adc_select_input(0);
        uint adc_x_raw = adc_read();
        adc_select_input(1);
        uint adc_y_raw = adc_read();

        if (adc_y_raw > 2000){
            if(adc_x_raw <1700) { // front left
                printf("3 section\n");
                direction_0=3 ;
                direction_1=3 ;
            }
            else if(adc_x_raw >3000){ // front right
                printf("1 section\n");
                direction_0=1 ;
                direction_1=1 ;
            }
            else {
                printf("2 section\n"); // front
                direction_0=2 ;
                direction_1=2 ;
            }
        }
        // back
        else {
            if(adc_x_raw <2000) { // back left
                printf("4 section\n");
                direction_0=4 ;
                direction_1=4 ;
            }
            else if(adc_x_raw >=2000){ // back right
                printf("0 section\n");
                direction_0=0 ;
                direction_1=0 ;
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