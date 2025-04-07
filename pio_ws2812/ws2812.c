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
 // Buffer to store LED color values for each pixel
 uint32_t led_buffer[NUM_PIXELS];

 // Flag indicating whether the timer is ready to trigger actions
 bool timer_ready = false;

 // Flag used to track whether a certain action has been played
 bool played = false;

 // Index for current LED being processed
 int led_index = 0;

 // Index used for sensor-related logic (implementation not shown here)
 int sensor_index = 0;
 
 // Predefined color values in RGB hex format
 const uint32_t green = 0xff00;
 const uint32_t white = 0xFFFFFF;
 const uint32_t blue = 0xFF;
 const uint32_t red = 0xFF0000;
 const uint32_t purple = 0x36013F;
 const uint32_t orange = 0xFFA500;

 // Array of colors for general display sequences
 uint32_t colours[3] = {blue, purple, green};

 // Alternate color sequence used in playback scenarios
 uint32_t play_through[3] = {blue, orange, purple};

 // Tracks the current index within a sequence
 uint current_index = 0;

 // Struct to define a sequence of three LED indices
 typedef struct{
     unsigned int first;
     unsigned int second;
     unsigned int third;
 } Sequence;

 // Default LED sequence (indices 0, 1, 2)
 Sequence led_sequence = {0,1,2};
#pragma endregion

#pragma region LED Methods
    // Timer callback function, executes on each timer tick
    bool timer_callback(struct repeating_timer *t) {
        printf("Timer callback executed\n");
        return true;  // Keeps the timer running
    }

    // Initializes and starts a repeating timer at specified interval
    void start_timer(int interval_ms) {
        struct repeating_timer timer;
        add_repeating_timer_ms(interval_ms, timer_callback, NULL, &timer);
    }

    // Sends a pixel value to the PIO state machine with GRB format adjustment
    static inline void put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
        pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
    }

    // Constructs a 32-bit color value from individual RGB components
    static inline uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
        // Combines RGB components into a single 32-bit integer
        // return (g<<24) | (b<<16) | (r);
        return (r) | (g) | (b);
        // return (b<<16) | (r<<8) | g | 0xFF000000;
        // return (g<<24) | (r<<16) | (b<<8) | 0x00;
    }

    // Converts a 24-bit RGB color to GRB format with padding
    uint32_t format_colour(uint32_t colour) {
        uint8_t r = (colour & 0xFF0000) >> 16;
        uint8_t g = (colour & 0x00FF00) >> 8;
        uint8_t b = (colour & 0x0000FF);
        return g<<24 | r<<16 | b<<8;
    }

    // Adjusts color brightness and returns formatted GRB 32-bit color
    uint32_t set_colour(uint32_t colour, float brightness) {
        uint8_t r = ((colour & 0xFF0000) >> 16) * brightness;
        uint8_t g = ((colour & 0x00FF00) >> 8) * brightness;
        uint8_t b = ((colour & 0x0000FF)) * brightness;
        return g<<24 | r<<16 | b<<8;
    }

    // Sets all LEDs on the strip to a given color
    void set_all_leds(PIO pio, uint sm, uint32_t color, int count) {
        for (int i = 0; i < count; i++) {
            pio_sm_put_blocking(pio, sm, color);
        }
    }

    // Sets a specific LED to a given color and refreshes the entire strip
    void set_led(PIO pio, uint sm, uint32_t color, int index) {
        led_buffer[index] = format_colour(color);
        for(int i=0; i<NUM_PIXELS; i++) {
            pio_sm_put_blocking(pio, sm, led_buffer[i]);
        }
    }

    // Refreshes the LED buffer without modifying the brightness (unused color parameter)
    void set_led_brightness(PIO pio, uint sm, uint32_t color, int index, float brightness) {
        for(int i=0; i<NUM_PIXELS; i++) {
            pio_sm_put_blocking(pio, sm, led_buffer[i]);
        }
    }

    // Sends a given color to the entire LED sequence for a specified count
    void sequence_leds(PIO pio, uint sm, uint32_t color, int count) {
        for (int i = 0; i < count; i++) {
            pio_sm_put_blocking(pio, sm, color);
        }
    }

    // Updates the sequence by shifting elements and adding a new index at the end
    void set_sequence(Sequence *seq, int new_index){
        seq->first = seq->second;
        seq->second = seq->third;
        seq->third = new_index;
    }

    // Applies a three-part color pattern to the LED strip based on the sequence
    void set_leds_in_sequence(Sequence s, PIO pio, uint sm){
        for (int i = 0; i < NUM_PIXELS; i++) {
            led_buffer[i] = 0;  // Clear the buffer
        }

        // Bright blue for the most recent index
        led_buffer[s.first] = set_colour(blue,0.95);

        // Purple for the second most recent, only if different from the first
        if (s.second != s.first) {
            led_buffer[s.second] = set_colour(purple,0.35);
        }

        // Dim white for the oldest in the sequence if distinct
        if (s.third != s.second && s.third != s.first) {
            led_buffer[s.third] = set_colour(white,0.09);
        }

        // Write all buffer values to the strip
        for(int i=0; i<NUM_PIXELS; i++) {
            pio_sm_put_blocking(pio, sm, led_buffer[i]);
        }
    }

    // Basic GPIO interrupt callback, currently logs the pin that triggered the event
    void gpio_callback(uint gpio, uint32_t events) {
        printf("GPIO %d interrupt\n", gpio);
    }

    // Computes the LED index based on a matrix layout (e.g., for 6 LEDs per strip)
    // Also sets the 'played' flag depending on the flag input
    int get_led_index(int strip, int led, int flag){
        if(flag == -1)
            played = false;
        else
            played = true;
        return ((strip - 1)*6) + led;
    }
    //  void clear_leds(PIO pio, uint sm) {
    //      for (int i = 0; i < sizeof(led_buffer); i++) {
    //          led_buffer[i] = 0;
    //      }
    //  }
    /*
    Example usage:
    set_sequence(led_sequence, led_index); // update the sequence with the new index
    */
#pragma endregion
 
//***************************************Sensor Stuff***************************************//
#pragma region PS Mux&SIG
// Define GPIO pins for the select lines (S0–S3) of MUX 1
#define MUX1_S0_PIN 0    
#define MUX1_S1_PIN 1  
#define MUX1_S2_PIN 2  
#define MUX1_S3_PIN 3

// Define GPIO pins for the select lines (S0–S3) of MUX 2
#define MUX2_S0_PIN 4  
#define MUX2_S1_PIN 5  
#define MUX2_S2_PIN 6  
#define MUX2_S3_PIN 7

// Define GPIO pins for the select lines (S0–S3) of MUX 3
#define MUX3_S0_PIN 9  
#define MUX3_S1_PIN 10  
#define MUX3_S2_PIN 11  
#define MUX3_S3_PIN 12

// Signal output pins from each multiplexer (likely analog signals routed to ADCs)
#define SIG_PIN1 28
#define SIG_PIN2 27
#define SIG_PIN3 26

//Mux 1 [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
//Mux 1 [F,Bb,Eb,Ab,C,F,Gb,B,E,A,Db,Gb,G,C,F,Bb]

//Mux 2 [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
//Mux 2 [D,G,Ab,Db,Gb,B,Eb,Ab,A,D,G,C,E,A,Bb,Eb]

//Mux 3 [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
//Mux 3 [Ab,Db,F,Bb,B,E,A,D,Gb,B,C,F,Bb,Eb,G,C]
#pragma endregion

#pragma region PS Consts
// ADC threshold below which a signal is considered a "press"
#define PRESSED_THRESHOLD 150  

// Number of stable reads required to confirm a sensor press
#define STABLE_READS 5  

// Delay (in microseconds) to allow the MUX to settle after switching channels
#define MUX_SETTLE_TIME_US 1800

// Number of channels per MUX (4-bit addressable: 2⁴ = 16)
#define MAX_CHANNELS 16  

// Duration (in milliseconds) to hold a chord before considering it finalized
#define CHORD_HOLD_TIME_MS 100  
#pragma endregion

#pragma region PS Variables
// Mutex for safely accessing shared sensor state in multithreaded context
static mutex_t sensor_mutex;

// Timestamp marking the last time ADC zeroing occurred (used for noise compensation or calibration)
static uint32_t last_adc_zero_time = 0;

// Double-buffered storage for sensor readings: [channel][buffer_index]
static int sensor_buffer[MAX_CHANNELS][2] = {{0}};

// Index to toggle between sensor buffer states (for stable read detection)
static int buffer_index = 0;

// Indicates which MUX is currently active (used to avoid crosstalk and manage switching)
volatile int active_mux = -1;

// Records the last MUX channel where a press was detected
int last_pressed_channel = -1;

// Timestamp when the last press was registered (used for debouncing or hold detection)
static uint32_t last_press_time = 0;

// Holds the channels detected as pressed in the previous cycle
static int last_detected_channels[MAX_CHANNELS] = {0};

// Number of channels detected as pressed in the previous scan
static int last_num_pressed = 0;

// Struct for encapsulating the configuration of a MUX (select pins, signal pin, and associated ADC channel)
typedef struct{
    uint8_t s0, s1, s2, s3;     // MUX select pins
    uint8_t sig_pin;            // Output signal pin from the MUX
    uint8_t adc_channel;        // Corresponding ADC channel connected to the signal pin
} MuxConfig;

// Array of MUX configurations, one entry per MUX unit
MuxConfig mux_configs[] = {
    {MUX1_S0_PIN, MUX1_S1_PIN, MUX1_S2_PIN, MUX1_S3_PIN, SIG_PIN1, 2}, // MUX1 feeds into ADC channel 2
    {MUX2_S0_PIN, MUX2_S1_PIN, MUX2_S2_PIN, MUX2_S3_PIN, SIG_PIN2, 1}, // MUX2 feeds into ADC channel 1
    {MUX3_S0_PIN, MUX3_S1_PIN, MUX3_S2_PIN, MUX3_S3_PIN, SIG_PIN3, 0}  // MUX3 feeds into ADC channel 0
};
#pragma endregion

#pragma region PS Methods

// Initializes the mutex used for thread-safe access to shared sensor state
void init_mutex(){
    mutex_init(&sensor_mutex);
}

// Initializes all GPIO pins associated with the select lines of the three multiplexers
void init_all_GPIO(){
    // MUX1 select lines
    gpio_init(MUX1_S0_PIN); gpio_set_dir(MUX1_S0_PIN, GPIO_OUT);
    gpio_init(MUX1_S1_PIN); gpio_set_dir(MUX1_S1_PIN, GPIO_OUT);
    gpio_init(MUX1_S2_PIN); gpio_set_dir(MUX1_S2_PIN, GPIO_OUT);
    gpio_init(MUX1_S3_PIN); gpio_set_dir(MUX1_S3_PIN, GPIO_OUT);

    // MUX2 select lines
    gpio_init(MUX2_S0_PIN); gpio_set_dir(MUX2_S0_PIN, GPIO_OUT);
    gpio_init(MUX2_S1_PIN); gpio_set_dir(MUX2_S1_PIN, GPIO_OUT);
    gpio_init(MUX2_S2_PIN); gpio_set_dir(MUX2_S2_PIN, GPIO_OUT);
    gpio_init(MUX2_S3_PIN); gpio_set_dir(MUX2_S3_PIN, GPIO_OUT);

    // MUX3 select lines
    gpio_init(MUX3_S0_PIN); gpio_set_dir(MUX3_S0_PIN, GPIO_OUT);
    gpio_init(MUX3_S1_PIN); gpio_set_dir(MUX3_S1_PIN, GPIO_OUT);
    gpio_init(MUX3_S2_PIN); gpio_set_dir(MUX3_S2_PIN, GPIO_OUT);
    gpio_init(MUX3_S3_PIN); gpio_set_dir(MUX3_S3_PIN, GPIO_OUT);
}

// Selects a specific channel on a given multiplexer by setting its control lines
void select_mux_channel(MuxConfig* mux, uint8_t channel) {
    gpio_put(mux->s0, (channel >> 0) & 1);
    gpio_put(mux->s1, (channel >> 1) & 1);
    gpio_put(mux->s2, (channel >> 2) & 1);
    gpio_put(mux->s3, (channel >> 3) & 1);
    // Allow time for the multiplexer signal to settle after switching
    busy_wait_us_32(MUX_SETTLE_TIME_US);
}

// Reads a stable average ADC value from the specified MUX's signal pin
uint16_t get_adc_value(MuxConfig* mux) {
    // Select the appropriate ADC input channel for this MUX
    adc_select_input(mux->adc_channel);
    
    // Discard the first read to avoid stale data
    adc_read();  

    uint16_t sum = 0;
    // Perform multiple reads and average them to reduce noise
    for (int i = 0; i < STABLE_READS; i++) {
        sum += adc_read();
        busy_wait_us_32(50);  // Small delay between reads for stability
    }
    return sum / STABLE_READS;  // Return averaged result
}

// Interrupt Service Routine (ISR) triggered when a signal pin detects a rising edge
void handle_sig_interrupt(uint gpio, uint32_t events){
    gpio_acknowledge_irq(gpio, events);  // Clear the interrupt flag
    for(int i = 0; i < 3; i++){
        // Identify which MUX triggered the interrupt
        if(gpio == mux_configs[i].sig_pin){
            active_mux = i;  // Set active MUX index for downstream processing
            break;
        }
    }
}

// Configures interrupts on each MUX's signal pin to detect rising edges
void init_gpio_interupts() {
    for (int i = 0; i < 3; i++) {
        gpio_init(mux_configs[i].sig_pin);               // Initialize the signal pin
        gpio_set_dir(mux_configs[i].sig_pin, GPIO_IN);   // Set as input
        gpio_pull_down(mux_configs[i].sig_pin);          // Ensure default state is low
        gpio_set_irq_enabled_with_callback(
            mux_configs[i].sig_pin,
            GPIO_IRQ_EDGE_RISE,
            true,
            &handle_sig_interrupt  // Attach ISR
        );
    }
}

// Stores a detected channel and timestamp in the sensor buffer
void update_sensor_buffer(int channel){
    sensor_buffer[buffer_index][0] = channel;
    sensor_buffer[buffer_index][1] = to_ms_since_boot(get_absolute_time());
    buffer_index = (buffer_index + 1) % MAX_CHANNELS;  // Wrap-around circular buffer
    active_mux = -1;  // Reset active MUX to prevent re-use until next interrupt
}

// Processes the sensor buffer and logs detected presses to console
void process_sensor_buffer(){
    for(int i = 0; i < MAX_CHANNELS; i++){
        if(sensor_buffer[i][0] != 0){
            printf("Channel %d was pressed at %d ms\n", sensor_buffer[i][0], sensor_buffer[i][1]);
            sensor_buffer[i][0] = 0;  // Reset after logging
        }
    }
}

#pragma endregion

 //***************************************Other Stuff***************************************//
// todo get free sm
PIO pio;       // Global handle for the PIO (Programmable I/O) hardware block
uint sm;       // State machine ID for PIO
uint offset;   // Offset into the PIO program (not used in this snippet)

// Multi-threaded function to be run on Core 1 of the Raspberry Pi Pico
void core1_entry() {
    while (true) {  // Infinite loop for continuous background processing
        for (int i = 0; i < 3; i++) {  // Iterate over all three multiplexers
            MuxConfig* current_mux = &mux_configs[i];  // Pointer to current MUX config
            adc_select_input(current_mux->adc_channel);  // Set the ADC to read from this MUX’s signal line

            for (int channel = 0; channel < MAX_CHANNELS; channel++) {  // Loop through all channels on this MUX
                select_mux_channel(current_mux, channel);  // Set the MUX to the specified channel
                sleep_ms(0);  // Minimal delay; this may be a placeholder or intentional no-op (could be removed)

                if (active_mux != -1) {  // If an interrupt has recently identified this MUX as active
                    printf("Processing signal from MUX %d...\n", active_mux + 1);
                    update_sensor_buffer(channel);  // Log the active channel and timestamp into the buffer
                }
            }

            // If no active MUX was detected in this scan cycle:
            if (active_mux == -1) {
                if (last_adc_zero_time == 0) {
                    // Start a timer for how long it's been since no channel was pressed
                    last_adc_zero_time = to_ms_since_boot(get_absolute_time());
                } else if (to_ms_since_boot(get_absolute_time()) - last_adc_zero_time >= 1000) {
                    // If it's been more than 1 second since no input detected, assume a chord release
                    // printf("Chord released. Resetting detection.\n");  // Optional debug message
                    process_sensor_buffer();  // Process and clear logged sensor activity
                    last_adc_zero_time = 0;  // Reset the timer to detect future idle periods
                }
            }
        }
    }
}
int main() {
    stdio_init_all();  // Initialize all standard IO, including USB serial if connected

    //***************************************LED Init***************************************//
    #pragma region LED INIT
        // Unload any existing WS2812 (NeoPixel) PIO program and free its state machine
        pio_remove_program_and_unclaim_sm(&ws2812_program, pio, sm, offset);

        // Load WS2812 program into a free PIO instance and assign a state machine
        // This variant selects a PIO that supports GPIOs >= 32, if needed by the board
        bool success = pio_claim_free_sm_and_add_program_for_gpio_range(
            &ws2812_program, &pio, &sm, &offset, WS2812_PIN, 1, true);
        hard_assert(success);  // Abort if the program failed to load (prevents undefined behavior)

        // Initialize the WS2812 driver with timing and pin configuration
        ws2812_program_init(pio, sm, offset, WS2812_PIN, 800000, IS_RGBW);

        // Initialize the LED buffer with all LEDs turned off (black)
        for (int i = 0; i < NUM_PIXELS; i++) {
            led_buffer[i] = 0;
        }

        // Color variables for later use (standard 0xRRGGBB format)
        int t = 0;
        uint32_t colour = 0x00FF00;     // Green
        uint32_t error = 0xFF0000;      // Red
        uint32_t colouroff = 0x000000;  // LED off

        // Select a random LED to be targeted initially
        led_index = rand() % ((NUM_PIXELS - 1) - 0 + 1) + 0;  // Random index from 0 to NUM_PIXELS-1
        printf("LED INDEX: %d\n", led_index);

        // Optionally set an LED here, currently done through sequence logic
        set_leds_in_sequence(led_sequence, pio, sm);  // Light up initial sequence
    #pragma endregion

    //***************************************ADC Init***************************************//
    #pragma region ADC INIT
        adc_init();               // Initialize the ADC hardware
        init_all_GPIO();         // Configure GPIOs for MUX control
        init_gpio_interupts();   // Enable GPIO interrupts on SIG lines
        init_mutex();            // Prepare sensor mutex for thread-safe access
        multicore_launch_core1(core1_entry);  // Launch signal processing loop on core 1
    #pragma endregion

    //***************************************Integration***************************************//
    printf("starting\n");
    
    // Wait until USB serial is connected (optional, useful for debugging)
    while (!stdio_usb_connected) {
        sleep_ms(100);  // Polling every 100 ms
    }

    // Main loop is empty — actual work happens in Core 1
    while (true) {
        // Possibly used for UI, status reporting, or event-driven tasks in the future
    }

    // Clean up: unload the WS2812 program and free associated resources (unlikely to be reached)
    pio_remove_program_and_unclaim_sm(&ws2812_program, pio, sm, offset);
}
