/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

 #include <stdio.h>
 #include <stdlib.h>
 #include <string.h>
 #include "pico/stdlib.h"
 #include "hardware/pio.h"
 #include "hardware/clocks.h"
 #include "ws2812.pio.h"
 #include "hardware/timer.h"
 #include "hardware/adc.h"
 #include "hardware/gpio.h"
 #include "pico/multicore.h"
 
 //****************************************LED  Stuff****************************************//
 /**
  * NOTE:
  *  Take into consideration if your WS2812 is a RGB or RGBW variant.
  *
  *  If it is RGBW, you need to set IS_RGBW to true and provide 4 bytes per 
  *  pixel (Red, Green, Blue, White) and use urgbw_u32().
  *
  *  If it is RGB, set IS_RGBW to false and provide 3 bytes per pixel (Red,
  *  Green, Blue) and use urgb_u32().
  *
  *  When RGBW is used with urgb_u32(), the White channel will be ignored (off).
  *
  */
 #define IS_RGBW false   // set to true if using RGBW WS2812
 #define NUM_PIXELS 6 // number of pixels in the strip
 
 #ifdef PICO_DEFAULT_WS2812_PIN
 #define WS2812_PIN PICO_DEFAULT_WS2812_PIN
 #else
 // default to pin 2 if the board doesn't have a default WS2812 pin defined
 #define WS2812_PIN 16
 #endif
 
 // Check the pin is compatible with the platform
 #if WS2812_PIN >= NUM_BANK0_GPIOS
 #error Attempting to use a pin>=32 on a platform that does not support it
 #endif
 
 
 uint32_t led_buffer[NUM_PIXELS];
 bool timer_ready = false;
 
 int led_index = 0;
 int sensor_index = 0;
 
 
 // Callback function to replace sleep
 bool timer_callback(struct repeating_timer *t) {
     printf("Timer callback executed\n");
     return true;  // Return true to keep the timer running
 }
 
 void start_timer(int interval_ms) {
     struct repeating_timer timer;
     add_repeating_timer_ms(interval_ms, timer_callback, NULL, &timer);
 }
 
 static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
     pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
 }
 
 static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
     // return (g<<24) | (b<<16) | (r);
     return (r) | (g) | (b);
     // return (b<<16) | (r<<8) | g | 0xFF000000;
     // return (g<<24) | (r<<16) | (b<<8) | 0x00;
 }
 
 uint32_t format_colour(uint32_t colour) {
     uint8_t r = (colour & 0xFF0000) >> 16;
     uint8_t g = (colour & 0x00FF00) >> 8;
     uint8_t b = (colour & 0x0000FF);
     return g<<24 | r<<16 | b<<8 ;//| 0xFF000000;
 }
 uint32_t set_colour(uint32_t colour, float brightness) {
     uint8_t r = ((colour & 0xFF0000) >> 16) * brightness;
     uint8_t g = ((colour & 0x00FF00) >> 8) * brightness;
     uint8_t b = ((colour & 0x0000FF)) * brightness;
     return g<<24 | r<<16 | b<<8 ;//| 0xFF000000;
 }

//  void clear_leds(PIO pio, uint sm) {
//      for (int i = 0; i < sizeof(led_buffer); i++) {
//          led_buffer[i] = 0;
//      }
//  }
 
 void set_all_leds(PIO pio, uint sm, uint32_t color, int count) {
     for (int i = 0; i < count; i++) {
         pio_sm_put_blocking(pio, sm, color);
     }
 }
 void set_led(PIO pio, uint sm, uint32_t color, int index) {
     led_buffer[index] = format_colour(color);
     for(int i=0; i<NUM_PIXELS; i++) {
         pio_sm_put_blocking(pio, sm, led_buffer[i]);
     }
 }
 void set_led_brightness(PIO pio, uint sm, uint32_t color, int index, float brightness) {
     for(int i=0; i<NUM_PIXELS; i++) {
         pio_sm_put_blocking(pio, sm, led_buffer[i]);
     }
 }
 void sequence_leds(PIO pio, uint sm, uint32_t color, int count) {
     for (int i = 0; i < count; i++) {
         pio_sm_put_blocking(pio, sm, color);
     }
 }
 
 //****************************************LED  Stuff****************************************//
 const uint32_t green = 0xff00;
 const uint32_t white = 0xFFFFFF;
 const uint32_t blue = 0xFF;
 const uint32_t red = 0xFF0000;
 const uint32_t purple = 0x36013F;
 const uint32_t orange = 0xFFA500;
 uint32_t colours[3] = {blue, purple, green};
 uint32_t play_through[3] = {blue, orange, purple};
 uint current_index = 0;
 
 //***************************************Sensor Stuff***************************************//
 #pragma region Mux&SIG
 #define MUX1_S0_PIN 0  
 #define MUX1_S1_PIN 1  
 #define MUX1_S2_PIN 2  
 #define MUX1_S3_PIN 3
 
 #define MUX2_S0_PIN 4  
 #define MUX2_S1_PIN 5  
 #define MUX2_S2_PIN 6  
 #define MUX2_S3_PIN 7
 
 #define MUX3_S0_PIN 9  
 #define MUX3_S1_PIN 10  
 #define MUX3_S2_PIN 11  
 #define MUX3_S3_PIN 12
 
 #define SIG_PIN1 28
 #define SIG_PIN2 27
 #define SIG_PIN3 26
#pragma endregion

 #define PRESSED_THRESHOLD 750  
 #define STABLE_READS 5  
 #define MUX_SETTLE_TIME_US 1800
 
 volatile int active_mux = -1;

 
typedef struct{
    uint8_t s0,s1,s2,s3;
    uint8_t sig_pin;
    uint8_t adc_channel; 
}MuxConfig;

MuxConfig mux_configs[] ={
    {MUX1_S0_PIN,MUX1_S1_PIN,MUX1_S2_PIN,MUX1_S3_PIN,SIG_PIN1,2},
    {MUX2_S0_PIN,MUX2_S1_PIN,MUX2_S2_PIN,MUX2_S3_PIN,SIG_PIN2,1},
    {MUX3_S0_PIN,MUX3_S1_PIN,MUX3_S2_PIN,MUX3_S3_PIN,SIG_PIN3,0}
};
void handle_sig1_interrupt(uint gpio, uint32_t events){
    gpio_acknowledge_irq(gpio, events);
    active_mux = 0;
}
void handle_sig2_interrupt(uint gpio, uint32_t events){
    gpio_acknowledge_irq(gpio, events);
    active_mux = 1;
}
void handle_sig3_interrupt(uint gpio, uint32_t events){
    gpio_acknowledge_irq(gpio, events);
    active_mux = 2;
}
 void select_mux_channel(MuxConfig* mux, uint8_t channel) {
    gpio_put(mux->s0, (channel >> 0) & 1);
    gpio_put(mux->s1, (channel >> 1) & 1);
    gpio_put(mux->s2, (channel >> 2) & 1);
    gpio_put(mux->s3, (channel >> 3) & 1);
    busy_wait_us_32(MUX_SETTLE_TIME_US);
    //sleep_us(MUX_SETTLE_TIME_US);  // Allow MUX to fully switch
}

uint16_t get_adc_value(MuxConfig* mux) {
   adc_select_input(mux->adc_channel);
    // Discard first reading (helps eliminate residual values)
    adc_read();  
    busy_wait_us_32(50); 
    //sleep_us(100);  

    uint16_t sum = 0;
    for (int i = 0; i < STABLE_READS; i++) {
        sum += adc_read();
        //sleep_us(50);
        busy_wait_us_32(50);
    }
    return sum / STABLE_READS;  
}
void init_gpio_interupts() {
    for (int i = 0; i < 3; i++) {
        gpio_init(mux_configs[i].sig_pin);
        gpio_set_dir(mux_configs[i].sig_pin, GPIO_IN);
        gpio_pull_down(mux_configs[i].sig_pin);
    }

    gpio_set_irq_enabled_with_callback(SIG_PIN1, GPIO_IRQ_EDGE_RISE, true, &handle_sig1_interrupt);
    //gpio_set_irq_enabled_with_callback(SIG_PIN2, GPIO_IRQ_EDGE_RISE, true, &handle_sig2_interrupt);
    //gpio_set_irq_enabled_with_callback(SIG_PIN3, GPIO_IRQ_EDGE_RISE, true, &handle_sig3_interrupt);
}
 
 //***************************************Sensor Stuff***************************************//
 //***************************************Other Stuff***************************************//
 typedef struct{
     unsigned int first;
     unsigned int second;
     unsigned int third;
 } Sequence;
 Sequence led_sequence = {0,1,2};
 
 void set_sequence(Sequence *seq){
     seq->first = seq->second;
     seq->second = seq->third;
     seq->third = rand() % NUM_PIXELS;
 }
 void set_leds_in_sequence(Sequence s, PIO pio, uint sm){
     for (int i = 0; i < NUM_PIXELS; i++) {
         led_buffer[i] = 0;
     }
     led_buffer[s.first] = set_colour(blue,0.95);
     if (s.second != s.first) {
         led_buffer[s.second] = set_colour(purple,0.35);
     }
     if (s.third != s.second && s.third != s.first) {
         led_buffer[s.third] = set_colour(white,0.09);
     }
     for(int i=0; i<NUM_PIXELS; i++) {
         pio_sm_put_blocking(pio, sm, led_buffer[i]);
     }
 }
 int last_pressed_channel = -1;  
 //Interrupts
 void gpio_callback(uint gpio, uint32_t events) {
     printf("GPIO %d interrupt\n", gpio);
 }
 
 //***************************************Other Stuff***************************************//
// todo get free sm
PIO pio;
uint sm;
uint offset;

#define MAX_CHANNELS 16  // Max channels per MUX
#define CHORD_HOLD_TIME_MS 100  // Time to allow finger placement

void process_mux_signal(int mux_index) {
    static uint32_t last_press_time = 0;  
    static int last_detected_channels[MAX_CHANNELS] = {0}; 
    static int last_num_pressed = 0;  

    printf("Processing signal from MUX %d...\n", mux_index + 1);

    MuxConfig* current_mux = &mux_configs[mux_index];
    adc_select_input(current_mux->adc_channel);

    int detected_channels[MAX_CHANNELS] = {0};  
    int num_pressed = 0;

    for (int channel = 0; channel < MAX_CHANNELS; channel++) {
        select_mux_channel(current_mux, channel);
        uint16_t value = get_adc_value(current_mux);

        if (value > PRESSED_THRESHOLD) {
            printf("Value: %d");
            detected_channels[num_pressed++] = channel;
        }
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());

    if (num_pressed > 0) {
        // If this is the first press, start the timer
        if (last_num_pressed == 0) {
            last_press_time = now;
        }
        printf("NumPressed : %d, Now: %d",num_pressed,now);
        // Wait until user has had time to place fingers
        if (now - last_press_time >= CHORD_HOLD_TIME_MS) {
            printf("MUX %d Detected Chord: ", mux_index + 1);
            for (int i = 0; i < num_pressed; i++) {
                printf("%d ", detected_channels[i]);
                last_detected_channels[i] = detected_channels[i]; 
            }
            printf("\n");
            last_num_pressed = num_pressed; 
        }
    } 
    // If no channels are pressed, only reset after full release
    else if (last_num_pressed > 0) {
        printf("Chord released. Resetting detection.\n");
        last_num_pressed = 0;
        memset(last_detected_channels, 0, sizeof(last_detected_channels));
    }
}

// void core1_entry(){
// }
 int main() {
     stdio_init_all();
     printf("starting\n");
     while(!stdio_usb_connected){
         sleep_ms(100);
     } 
     //***************************************LED Init***************************************//

 
     pio_remove_program_and_unclaim_sm(&ws2812_program, pio, sm, offset);
     // This will find a free pio and state machine for our program and load it for us
     // We use pio_claim_free_sm_and_add_program_for_gpio_range (for_gpio_range variant)
     // so we will get a PIO instance suitable for addressing gpios >= 32 if needed and supported by the hardware
     bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &pio, &sm, &offset, WS2812_PIN, 1, true);
     hard_assert(success);
 
     ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);
 
     // Initialize the buffer to off (0)
     for (int i = 0; i < NUM_PIXELS; i++) {
         led_buffer[i] = 0;
     }
     
     int t = 0;
     uint32_t colour = 0x00FF00; // Red color (0xRRGGBB)
     uint32_t error = 0xFF0000; // Red color (0xRRGGBB)
     uint32_t colouroff = 0x000000; // Off color (0xRRGGBB)
 
     //***************************************LED Init***************************************//
 
     //***************************************ADC Init***************************************//
    #pragma region GPIO INIT // MUX1
    gpio_init(MUX1_S0_PIN); gpio_set_dir(MUX1_S0_PIN, GPIO_OUT);
    gpio_init(MUX1_S1_PIN); gpio_set_dir(MUX1_S1_PIN, GPIO_OUT);
    gpio_init(MUX1_S2_PIN); gpio_set_dir(MUX1_S2_PIN, GPIO_OUT);
    gpio_init(MUX1_S3_PIN); gpio_set_dir(MUX1_S3_PIN, GPIO_OUT);

    // MUX2
    gpio_init(MUX2_S0_PIN); gpio_set_dir(MUX2_S0_PIN, GPIO_OUT);
    gpio_init(MUX2_S1_PIN); gpio_set_dir(MUX2_S1_PIN, GPIO_OUT);
    gpio_init(MUX2_S2_PIN); gpio_set_dir(MUX2_S2_PIN, GPIO_OUT);
    gpio_init(MUX2_S3_PIN); gpio_set_dir(MUX2_S3_PIN, GPIO_OUT);

    // MUX3
    gpio_init(MUX3_S0_PIN); gpio_set_dir(MUX3_S0_PIN, GPIO_OUT);
    gpio_init(MUX3_S1_PIN); gpio_set_dir(MUX3_S1_PIN, GPIO_OUT);
    gpio_init(MUX3_S2_PIN); gpio_set_dir(MUX3_S2_PIN, GPIO_OUT);
    gpio_init(MUX3_S3_PIN); gpio_set_dir(MUX3_S3_PIN, GPIO_OUT);
    #pragma endregion
    adc_init();
    init_gpio_interupts();
 
     // int last_pressed_channel = -1;  
     led_index = rand() % ((NUM_PIXELS-1) - 0 + 1) + 0; //random number between 0 and NUM_PIXELS - 1
     printf("LED INDEX: %d\n", led_index);
     // set_led_brightness(pio, sm, purple, led_index,0.4);
     set_leds_in_sequence(led_sequence, pio, sm);
     int i =0;
     //***************************************ADC Init***************************************//
     //***************************************Integration***************************************//
     //multicore_launch_core1(core1_entry);
     while(true){
        // if(multicore_fifo_rvalid){
        //     uint32_t msg = multicore_fifo_pop_blocking();
        //     // printf("Chanell %d pressed",msg);
        // }
        if(active_mux != -1){
            printf("\n%d",i++);
            process_mux_signal(active_mux);
            active_mux = -1;
            printf("\nProcessed Signal, Active Mus: %d",active_mux);
        }
        sleep_ms(1);
    }
     // This will free resources and unload our program
     pio_remove_program_and_unclaim_sm(&ws2812_program, pio, sm, offset);
 }
 