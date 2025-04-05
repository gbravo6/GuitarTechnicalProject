/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#pragma region Includes
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
    #include "pico/mutex.h"
 #pragma endregion
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
 
 //****************************************LED  Stuff****************************************//
 #pragma region LED Variables
    uint32_t led_buffer[NUM_PIXELS];
    bool timer_ready = false;
    bool played = false;

    int led_index = 0;
    int sensor_index = 0;
    
    const uint32_t green = 0xff00;
    const uint32_t white = 0xFFFFFF;
    const uint32_t blue = 0xFF;
    const uint32_t red = 0xFF0000;
    const uint32_t purple = 0x36013F;
    const uint32_t orange = 0xFFA500;
    uint32_t colours[3] = {blue, purple, green};
    uint32_t play_through[3] = {blue, orange, purple};
    uint current_index = 0;
    
    typedef struct{
        unsigned int first;
        unsigned int second;
        unsigned int third;
    } Sequence;
    Sequence led_sequence = {0,1,2};
#pragma endregion

#pragma region LED Methods
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

    void set_sequence(Sequence *seq, int new_index){
        seq->first = seq->second;
        seq->second = seq->third;
        seq->third = new_index;
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
    //Interrupts
    void gpio_callback(uint gpio, uint32_t events) {
        printf("GPIO %d interrupt\n", gpio);
    }
    int get_led_index(int strip, int led, int flag){
        if(flag == -1)
            played = false;
        else
            played = true;
        return ((strip - 1)*6) + led;
    }
    /*
    set_sequence(led_sequence, led_index); // update the sequence with the new index
    */

#pragma endregion
 
//***************************************Sensor Stuff***************************************//
 #pragma region PS Mux&SIG
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
#pragma region PS Consts

 #define PRESSED_THRESHOLD 250  
 #define STABLE_READS 5  
 #define MUX_SETTLE_TIME_US 1800
 #define MAX_CHANNELS 16  // Max channels per MUX
 #define CHORD_HOLD_TIME_MS 100  // Time to allow finger placement

 #pragma endregion
 #pragma region PS Variables
    static mutex_t sensor_mutex;
    static uint32_t last_adc_zero_time =0;
    static int sensor_buffer[MAX_CHANNELS][2]={{0}};
    static int buffer_index =0;
    volatile int active_mux = -1;
    int last_pressed_channel = -1;  
    
    static uint32_t last_press_time = 0;  
    static int last_detected_channels[MAX_CHANNELS] = {0}; 
    static int last_num_pressed = 0;  

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
    
 #pragma endregion
 #pragma region PS Methods
    void init_mutex(){
        mutex_init(&sensor_mutex);
    }
    void init_all_GPIO(){
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
    }
    void select_mux_channel(MuxConfig* mux, uint8_t channel) {
        gpio_put(mux->s0, (channel >> 0) & 1);
        gpio_put(mux->s1, (channel >> 1) & 1);
        gpio_put(mux->s2, (channel >> 2) & 1);
        gpio_put(mux->s3, (channel >> 3) & 1);
        busy_wait_us_32(MUX_SETTLE_TIME_US);
    }
    uint16_t get_adc_value(MuxConfig* mux) {
        adc_select_input(mux->adc_channel);
        // Discard first reading (helps eliminate residual values)
        adc_read();  

        uint16_t sum = 0;
        for (int i = 0; i < STABLE_READS; i++) {
            sum += adc_read();
            //sleep_us(50);
            busy_wait_us_32(50);
        }
        return sum / STABLE_READS;  
    }
    void handle_sig_interrupt(uint gpio, uint32_t events){
        gpio_acknowledge_irq(gpio, events);
        for(int i = 0; i<3;i++){
            if(gpio == mux_configs[i].sig_pin){
                active_mux = i;
                break;
            }
        }
    }
    void init_gpio_interupts() {
        for (int i = 0; i < 3; i++) {
            gpio_init(mux_configs[i].sig_pin);
            gpio_set_dir(mux_configs[i].sig_pin, GPIO_IN);
            gpio_pull_down(mux_configs[i].sig_pin);
            gpio_set_irq_enabled_with_callback(mux_configs[i].sig_pin, GPIO_IRQ_EDGE_RISE, true, &handle_sig_interrupt);
        }
    }
    void update_sensor_buffer(int channel){
        sensor_buffer[buffer_index][0] = channel;
        sensor_buffer[buffer_index][1] = to_ms_since_boot(get_absolute_time());
        buffer_index = (buffer_index + 1) % MAX_CHANNELS;
        active_mux = -1;
    }
    void process_sensor_buffer(){
        for(int i = 0; i< MAX_CHANNELS; i++){
            if(sensor_buffer[i][0]!=0){
                printf("Channel %d was pressed at %d ms\n",sensor_buffer[i][0],sensor_buffer[i][1]);
                sensor_buffer[i][0]=0;
            }
        }
    }
    // void process_mux_signal() {
    //     mutex_enter_blocking(&sensor_mutex);
        
        
    //     mutex_exit(&sensor_mutex);
    //     active_mux=-1;
    // }
    
    
    
#pragma endregion

 //***************************************Other Stuff***************************************//
// todo get free sm
PIO pio;
uint sm;
uint offset;

//Multi Thread func if needed
void core1_entry() {
    while (true) {
        for (int i = 0; i < 3; i++) {
            MuxConfig* current_mux = &mux_configs[i];
            adc_select_input(current_mux->adc_channel);

            for (int channel = 0; channel < MAX_CHANNELS; channel++) {
                select_mux_channel(current_mux, channel);
                sleep_ms(0);
                
                if (active_mux != -1) {
                    printf("Processing signal from MUX %d...\n", active_mux + 1);
                    update_sensor_buffer(channel);
                }
            }
            if (active_mux == -1) {
                if (last_adc_zero_time == 0) {
                    last_adc_zero_time = to_ms_since_boot(get_absolute_time());
                } else if (to_ms_since_boot(get_absolute_time()) - last_adc_zero_time >= 1000) {
                    //printf("Chord released. Resetting detection.\n");
                    process_sensor_buffer();
                    last_adc_zero_time = 0; // Reset timer
                }
            }
        }
    }
}

int main() {
     stdio_init_all();
     //***************************************LED Init***************************************//
     #pragma region LED INIT
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

        
        // int last_pressed_channel = -1;  
        led_index = rand() % ((NUM_PIXELS-1) - 0 + 1) + 0; //random number between 0 and NUM_PIXELS - 1
        printf("LED INDEX: %d\n", led_index);
        // set_led_brightness(pio, sm, purple, led_index,0.4);
        set_leds_in_sequence(led_sequence, pio, sm);
     #pragma endregion
 
     //***************************************ADC Init***************************************//
    #pragma region ADC INIT
        adc_init();
        init_all_GPIO();
        init_gpio_interupts();
        init_mutex();
        multicore_launch_core1(core1_entry);
    #pragma endregion
    
     //***************************************Integration***************************************//
     printf("starting\n");
     while(!stdio_usb_connected){
         sleep_ms(100);
     } 
     //multicore_launch_core1(core1_entry);
     while(true){
        // if(active_mux != -1){
        //     process_mux_signal();
        // }
    }
     // This will free resources and unload our program
     pio_remove_program_and_unclaim_sm(&ws2812_program, pio, sm, offset);
}