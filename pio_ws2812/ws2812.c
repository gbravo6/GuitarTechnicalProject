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
    #include "pico/cyw43_arch.h"
    #include "lwip/udp.h"
    #include "lwip/pbuf.h"
 #pragma endregion

 //*****************************************WIFI/UDP*****************************************//
 #pragma region Wifi Defines
 #define WIFI_NAME "JennyTalls"
 #define PASSWORD  "test12345"
 #define UDP_PORT 1666
 #define PC_IP "" //define on testing
 volatile bool start_chord_learning = false;
 typedef enum {
    WAITING_FOR_CHORD_COMMAND,
    WAITING_FOR_CHORD_NAME,
    LEARNING_CHORD
} ChordLearnState;

volatile ChordLearnState chord_learn_state = WAITING_FOR_CHORD_COMMAND;
 #pragma endregion
 
#pragma region Bools
bool chord_match = false; // Flag to indicate if a chord is pressed
bool played = false;  // Flag used to track whether a certain action has been played
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
 #define NUM_PIXELS 48 // number of pixels in the strip
 
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

 // Index for current LED being processed
 int led_index = 0;

 // Index used for sensor-related logic (implementation not shown here)
 int sensor_index = 0;
 
 // Predefined color values in RGB hex format
 #define GREEN 0xff00
 #define WHITE 0xFFFFFF
 #define BLUE  0xFF
 #define RED  0xFF0000
 #define PURPLE 0x36013F
 #define ORANGE 0xFFA500
 #define YELLOW 0xFFFF00
 #define OFF    0x000000
 // Array of colors for general display sequences
 uint32_t colours[3] = {BLUE, PURPLE, GREEN};

 // Alternate color sequence used in playback scenarios
 uint32_t play_through[3] = {BLUE, ORANGE, PURPLE};

 // Tracks the current index within a sequence
 uint current_index = 0;

 // Struct to define a sequence of three LED indices
 typedef struct{
     unsigned int first;
     unsigned int second;
     int third;
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
    void all_on(PIO pio, uint sm){
        for (int i = 0; i < NUM_PIXELS; i++) {
            led_buffer[i] = set_colour(GREEN,0.95);
        }
        for(int i=0; i<NUM_PIXELS; i++) {
            pio_sm_put_blocking(pio, sm, led_buffer[i]);
        }
    }
    // Applies a three-part color pattern to the LED strip based on the sequence
    void set_leds_in_sequence(Sequence s, PIO pio, uint sm){
        for (int i = 0; i < NUM_PIXELS; i++) {
            led_buffer[i] = 0;  // Clear the buffer
        }

        // Bright blue for the most recent index
        led_buffer[s.first] = set_colour(BLUE,0.95);

        // Purple for the second most recent, only if different from the first
        if (s.second != s.first) {
            led_buffer[s.second] = set_colour(PURPLE,0.35);
        }

        // Dim white for the oldest in the sequence if distinct
        if (s.third != s.second && s.third != s.first) {
            led_buffer[s.third] = set_colour(WHITE,0.09);
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
    int get_led_index(int mx, int ch, int flag){
        if(flag == -1)
            played = false;
        else
            played = true;
        if(mx == 1){
            return ch;;
        }
        if(mx == 2){
            return ch + 16;
        }
        if(mx == 3){
            return ch + 32;
        }
    }
     void clear_leds(PIO pio, uint sm) {
         for (int i = 0; i < NUM_PIXELS; i++) {
             led_buffer[i] = 0;
         }
     }
    /*
    Example usage:
    set_sequence(led_sequence, led_index); // update the sequence with the new index
    */
    //TEST CODE
//    void test_all_colors(PIO pio, uint sm, int delay_ms) {
//         uint32_t colors[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFFFF};  // Red, Green, Blue, White
//         int num_colors = sizeof(colors) / sizeof(colors[0]);
//         for (int c = 0; c < num_colors; c++) {
//             for (int i = 0; i < NUM_PIXELS; i++) {
//                 led_buffer[i] = set_colour(colors[c], 0.8);
//             }
//             for (int i = 0; i < NUM_PIXELS; i++) {
//                 pio_sm_put_blocking(pio, sm, led_buffer[i]);
//             }
//             sleep_ms(delay_ms);
//         }
//     }
//     void test_each_led(PIO pio, uint sm, uint32_t color, int delay_ms) {
//         for (int i = 0; i < NUM_PIXELS; i++) {
//             for (int j = 0; j < NUM_PIXELS; j++) {
//                 led_buffer[j] = (j == i) ? set_colour(color, 0.9) : 0x000000;
//             }
//             for (int j = 0; j < NUM_PIXELS; j++) {
//                 pio_sm_put_blocking(pio, sm, led_buffer[j]);
//             }
//             sleep_ms(delay_ms);
//         }
//     }
    
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

#define UNUSED_CHANNEL -1
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

#define MAX_MUX_CHANNELS 3
#define MAX_CHORD_CHANNELS 3  // Max number of channels per mux for a chord

#pragma endregion

#pragma region PS Variables

typedef struct {
    int mux_id;              // 0-indexed: MUX1 = 0, MUX2 = 1, etc.
    int channels[MAX_CHORD_CHANNELS];
    int num_channels;
} MuxChannelGroup;

typedef struct {
    const char* name;
    MuxChannelGroup mux_channels[MAX_MUX_CHANNELS];
    int num_mux_groups;
} Chord;

// Chord Definitions
Chord chord_A = {
    "Ch-A",
    {
        {0, {8, 9, 10}, 3}
    },
    1
};

Chord chord_Am = {
    "Ch-Am",
    {
        {0, {4, 8, 9}, 3}
    },
    1
};

Chord chord_C = {
    "Ch-C",
    {
        {0, {4, 8, 13}, 3}
    },
    1
};

Chord chord_D = {
    "Ch-D",
    {
        {0, {9, 11}, 2},
        {1, {0}, 1}
    },
    2
};

Chord chord_Dm = {
    "Ch-Dm",
    {
        {0, {5, 9}, 2},
        {1, {0}, 1}
    },
    2
};

Chord chord_E = {
    "Ch-E",
    {
        {0, {3, 7, 8}, 3}
    },
    1
};

Chord chord_Em = {
    "Ch-Em",
    {
        {0, {7, 8}, 2}
    },
    1
};

Chord chord_G = {
    "Ch-G",
    {
        {0, {7, 12}, 2},
        {1, {0}, 1}
    },
    2
};

// Lookup table
Chord* chord_library[] = {
    &chord_A, &chord_Am, &chord_C, &chord_D,
    &chord_Dm, &chord_E, &chord_Em, &chord_G
};

const int chord_library_size = sizeof(chord_library) / sizeof(chord_library[0]);

// Mutex for safely accessing shared sensor state in multithreaded context
static mutex_t sensor_mutex;

// Timestamp marking the last time ADC zeroing occurred (used for noise compensation or calibration)
static uint32_t last_adc_zero_time = 0;

// Double-buffered storage for sensor readings: [channel][buffer_index]
static int sensor_buffer[MAX_CHANNELS][2] = {{UNUSED_CHANNEL, 0}};
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
bool sensor_state[MAX_MUX_CHANNELS][MAX_CHANNELS] = {false};

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
    //active_mux = -1;  // Reset active MUX to prevent re-use until next interrupt
}

void process_sensor_buffer(){
    for(int i = 0; i < MAX_CHANNELS; i++){
        if(sensor_buffer[i][0] != UNUSED_CHANNEL){
            printf("Channel %d was pressed at %d ms\n", sensor_buffer[i][0], sensor_buffer[i][1]);
            sensor_buffer[i][0] = UNUSED_CHANNEL;  // Reset correctly
        }
    }
}
bool is_chord_pressed_correctly(const Chord* chord, bool sensor_state[MAX_MUX_CHANNELS][MAX_CHANNELS]) {
    for (int i = 0; i < chord->num_mux_groups; ++i) {
        MuxChannelGroup group = chord->mux_channels[i];
        for (int j = 0; j < group.num_channels; ++j) {
            int ch = group.channels[j];
            if (!sensor_state[group.mux_id][ch]) {
                return false;  // Required channel not pressed
            }
        }
    }
    return true;  // All required channels pressed
}
bool are_all_sensors_released(bool sensor_state[MAX_MUX_CHANNELS][MAX_CHANNELS]) {
    for (int mux = 0; mux < MAX_MUX_CHANNELS; ++mux) {
        for (int ch = 0; ch < MAX_CHANNELS; ++ch) {
            if (sensor_state[mux][ch]) return false;
        }
    }
    return true;
}

Chord* current_target_chord = NULL;


//Run this FOO once User selects "Learn Chords."
// void run_chord_learning_mode(void) {
//     if (current_target_chord != NULL) {
//         //Debug
//         printf("Current TargetChord: %d == SensorState: %d",current_target_chord,sensor_state);

//         if (is_chord_pressed_correctly(current_target_chord, sensor_state)) {
//             printf("✅ Correct chord %s played!\n", current_target_chord->name);

//             // Advance to next chord
//             if (current_target_chord != NULL) {
//                 printf("🎯 Next target chord: %s\n", current_target_chord->name);
//             } else {
//                 printf("🎉 All chords completed! Returning to menu...\n");
//             }
//             current_target_chord = NULL

//         }
//     } else {
//         // Start chord learning sequence if not initialized
//         if (current_target_chord != NULL) {
//             printf("🔰 Starting with chord: %s\n", current_target_chord->name);
//         }
//     }
// }
void run_chord_learning_mode(void) {
    printf("🔰 Learning chord: %s\n", current_target_chord->name);

    bool chord_detected = false;
    uint64_t last_release_time = 0;
    const uint64_t RELEASE_HOLD_MS = 600; // How long to wait for release

    while (1) {
        update_sensor_state();

        if (!chord_detected) {
            // Wait for correct chord to be pressed
            if (is_chord_pressed_correctly(current_target_chord, sensor_state)) {
                printf("✅ Correct chord %s pressed! Now release...\n", current_target_chord->name);
                chord_detected = true;
                last_release_time = 0; // Reset release timer
            }
        } else {
            // Wait for all sensors to be released
            if (are_all_sensors_released(sensor_state)) {
                if (last_release_time == 0) {
                    last_release_time = to_ms_since_boot(get_absolute_time());
                } else if (to_ms_since_boot(get_absolute_time()) - last_release_time >= RELEASE_HOLD_MS) {
                    printf("🎉 Chord learning for %s complete!\n", current_target_chord->name);
                    break; // Success! Exit loop
                }
            } else {
                last_release_time = 0; // Still pressed, reset timer
            }
        }
        sleep_ms(10); // Polling interval
    }
}


void update_sensor_state() {
    for (int mux_idx = 0; mux_idx < MAX_MUX_CHANNELS; mux_idx++) {
        MuxConfig* mux = &mux_configs[mux_idx];
        adc_select_input(mux->adc_channel);

        for (int ch = 0; ch < MAX_CHANNELS; ch++) {
            select_mux_channel(mux, ch);
            uint16_t value = get_adc_value(mux);
            sensor_state[mux_idx][ch] = value > PRESSED_THRESHOLD;
            busy_wait_us_32(200);  // Optional: debounce delay
        }
    }
}
//OG Sensor FOO(){
//// while (true) {  // Infinite loop for continuous background processing
    //     for (int i = 0; i < 3; i++) {  // Iterate over all three multiplexers
    //         MuxConfig* current_mux = &mux_configs[i];  // Pointer to current MUX config
    //         adc_select_input(current_mux->adc_channel);  // Set the ADC to read from this MUX’s signal line

    //         for (int channel = 0; channel < MAX_CHANNELS; channel++) {  // Loop through all channels on this MUX
    //             select_mux_channel(current_mux, channel);  // Set the MUX to the specified channel
    //             sleep_ms(5);  // Minimal delay; this may be a placeholder or intentional no-op (could be removed)

    //             if (active_mux != -1) {  // If an interrupt has recently identified this MUX as active
    //                 printf("Processing signal from MUX %d...\n", active_mux + 1);
    //                 update_sensor_buffer(channel);  // Log the active channel and timestamp into the buffer
    //             }
    //         }

    //         // If no active MUX was detected in this scan cycle:
    //         if (active_mux == -1) {
    //             if (last_adc_zero_time == 0) {
    //                 // Start a timer for how long it's been since no channel was pressed
    //                 last_adc_zero_time = to_ms_since_boot(get_absolute_time());
    //             } else if (to_ms_since_boot(get_absolute_time()) - last_adc_zero_time >= 1000) {
    //                 // If it's been more than 1 second since no input detected, assume a chord release
    //                 // printf("Chord released. Resetting detection.\n");  // Optional debug message
    //                 process_sensor_buffer();  // Process and clear logged sensor activity
    //                 last_adc_zero_time = 0;  // Reset the timer to detect future idle periods
    //             }
    //         }
    //     }
    // }
#pragma endregion
//**************************************More UDP Stuff**************************************//
#pragma region UDP Methods
//  void receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port){
//     printf("Receive Callback Reached.\n");

//     //convert databytes into char array/string
//     char *pData = (char *)p -> payload;

//     //remove trailing characters
//     char *pos = strchr(pData, '}');
//     if (pos != NULL){
//         *(pos + 1) = '\0';
//     }

//     printf("Received: %s\n", pData);
//     //printf ("Rcv from %s:%d, total %d [", addr, port, p->tot_len);

//     //free/reset buffer
//     pbuf_free(p);
// }

// void receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port){
//     printf("Receive Callback Reached.\n");

//     char *pData = (char *)p->payload;
//     char buffer[32] = {0};
//     strncpy(buffer, pData, sizeof(buffer) - 1);

//     // Clean up buffer
//     char *pos = strchr(buffer, '.');
//     if (pos != NULL) *(pos) = '\0';

//     printf("Received Chord Request: %s\n", buffer);

//     // Match incoming chord string to a Chord struct
//     current_target_chord = NULL;
//     for (int i = 0; i < chord_library_size; ++i) {
//         if (strcmp(buffer, chord_library[i]->name) == 0) {
//             printf("Chord matched: %s\n", current_target_chord->name);
//             chord_match = true;  // Set the chord match flag
//             if(chord_match){
//                 if(!played){
//                     current_target_chord = chord_library[i];
//                     printf("🎯 Target chord: %s\n", current_target_chord->name);
//                 }
//             }
//             break;
//         }
//     }
//     if (current_target_chord == NULL) {
//         printf("No matching chord found.\n");
//         chord_match = false;  // Reset the chord match flag
//     }
//     pbuf_free(p);
// }
void receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port){
    char *pData = (char *)p->payload;
    char buffer[32] = {0};
    strncpy(buffer, pData, sizeof(buffer) - 1);

    char *pos = strchr(buffer, '.');
    if (pos != NULL) *(pos) = '\0';

    printf("Received: %s (state=%d)\n", buffer, chord_learn_state);

    if (chord_learn_state == WAITING_FOR_CHORD_COMMAND) {
        if (strcmp(buffer, "Chord") == 0) {
            chord_learn_state = WAITING_FOR_CHORD_NAME;
            printf("🎸 Ready to receive chord name...\n");
        }
    } else if (chord_learn_state == WAITING_FOR_CHORD_NAME) {
        // Find the chord by name
        current_target_chord = NULL;
        for (int i = 0; i < chord_library_size; ++i) {
            if (strcmp(buffer, chord_library[i]->name) == 0) {
                current_target_chord = chord_library[i];
                printf("Chord matched: %s\n", current_target_chord->name);
                chord_learn_state = LEARNING_CHORD;
                break;
            }
        }
        if (current_target_chord == NULL) {
            printf("No matching chord found for '%s'.\n", buffer);
            chord_learn_state = WAITING_FOR_CHORD_COMMAND; // Reset to wait for next "Chord."
        }
    }
    pbuf_free(p);
}
void udp_begin_receiving()
{
    //create new udp listener
    struct udp_pcb *listener = udp_new();
    //bind to ip address
    err_t err = udp_bind(listener, IP_ADDR_ANY, UDP_PORT);
    //begin receiving thread
    udp_recv(listener, receive_callback, NULL);
}

int wifi_init(){
    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // Enable wifi station
    cyw43_arch_enable_sta_mode();

    //  Connect to Wifi
    printf("Connecting to Wi-Fi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_NAME, PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect.\n");
        return 1;
    } else {
        printf("Connected.\n");
        // Read the ip address in a human readable way
        uint8_t *ip_address = (uint8_t*)&(cyw43_state.netif[0].ip_addr.addr);
        printf("IP address %d.%d.%d.%d\n", ip_address[0], ip_address[1], ip_address[2], ip_address[3]);
    }
}
 #pragma endregion
//***************************************DEMO STUFF*****************************************//
#define TOTAL_MUXES 3
#define TOTAL_CHANNELS 16
bool was_pressed[TOTAL_MUXES][TOTAL_CHANNELS] = {false};
// PIO + state machine should be passed or set globally
extern PIO pio;
extern uint sm;

typedef struct {
    int mux;
    int channel;
    char* note;
} NoteInstruction;

// Define your demo notes as provided
NoteInstruction demo_notes[] = {
    {1, 0,  "F"},
    {1, 12, "G"},
    {1, 7,  "B"},
    {1, 13, "C"},
    {1, 8,  "E"},
    {1, 14, "F"},
    {1, 9,  "A"},
    {1, 4,  "C"},
    {2, 0,  "D"},
    {1, 5,  "F"},
    {2, 1,  "G"}
};

#define NUM_DEMO_NOTES (sizeof(demo_notes) / sizeof(demo_notes[0]))

// Sets current and next note colors
void indicate_note_colors(int current, int next) {
    for (int i = 0; i < NUM_PIXELS; i++) led_buffer[i] = OFF;

    if (current >= 0 && current < NUM_DEMO_NOTES) {
        int led_idx = get_led_index(demo_notes[current].mux, demo_notes[current].channel, -1);
        led_buffer[led_idx] = set_colour(GREEN, 0.9);
    }
    if (next >= 0 && next < NUM_DEMO_NOTES) {
        int led_idx = get_led_index(demo_notes[next].mux, demo_notes[next].channel, -1);
        led_buffer[led_idx] = set_colour(YELLOW, 0.4);
    }

    for (int i = 0; i < NUM_PIXELS; i++) {
        pio_sm_put_blocking(pio, sm, led_buffer[i]);
    }
}

// Debounced and smoothed ADC read
uint16_t stable_adc_read(MuxConfig* mux, int channel) {
    select_mux_channel(mux, channel);
    uint32_t total = 0;
    for (int i = 0; i < 3; i++) {
        total += get_adc_value(mux);
        sleep_ms(1);
    }
    return total / 3;
}
// Check if correct note is pressed via ADC polling
bool is_note_pressed(int mux, int channel) {
    MuxConfig* current_mux = &mux_configs[mux - 1];  
    select_mux_channel(current_mux, channel);
    uint16_t value = get_adc_value(current_mux);
    if(value>PRESSED_THRESHOLD){
        printf("MUX %d Channel %d ADC value: %d\n", mux, channel, value);
    }
    return value > PRESSED_THRESHOLD;  // You should define PRESS_THRESHOLD appropriately
}

void Demo() {
    printf("Starting Demo mode...\n");

    int current_index = 0;
    uint64_t last_adc_zero_time = 0;

    while (current_index < NUM_DEMO_NOTES) {
        NoteInstruction current_note = demo_notes[current_index];
        NoteInstruction next_note = (current_index + 1 < NUM_DEMO_NOTES) ? demo_notes[current_index + 1] : (NoteInstruction){-1, -1, ""};

        indicate_note_colors(current_index, current_index + 1);  // ✅ Show current only when ready

        bool note_detected = false;

        while (!note_detected) {
            for (int m = 0; m < 3; m++) {
                MuxConfig* current_mux = &mux_configs[m];
                adc_select_input(current_mux->adc_channel);

                for (int ch = 0; ch < MAX_CHANNELS; ch++) {
                    select_mux_channel(current_mux, ch);
                    sleep_ms(5);  // Debounce

                    if (active_mux != -1) {
                        printf("Detected press: MUX %d, CH %d\n", active_mux + 1, ch);
                        update_sensor_buffer(ch);

                        if ((active_mux + 1) == current_note.mux && ch == current_note.channel) {
                            printf("✅ Correct note %s pressed\n", current_note.note);
                            //Debugging:
                            printf("Active_mux: %d == Current mux: %d\n",active_mux+1,current_note.mux);
                            printf("Channel: %d == current channel: %d\n",ch,current_note.channel);
                            int led_idx = get_led_index(current_note.mux, current_note.channel, -1);
                            printf("LED index: %d", led_idx);
                            led_buffer[led_idx] = set_colour(BLUE, 1.0);
                            for (int l = 0; l < NUM_PIXELS; l++) {
                                pio_sm_put_blocking(pio, sm, led_buffer[l]);
                            }
                            sleep_ms(200);
                            clear_leds(pio, sm);
                            note_detected = true;

                        } else {
                            printf("❌ Incorrect press at MUX %d, CH %d\n", active_mux + 1, ch);
                            //Debugging
                            printf("Active_mux: %d == Current mux: %d\n",active_mux+1,current_note.mux);
                            printf("Channel: %d == current Mux: %d\n",ch,current_note.channel);
                            int led_idx = get_led_index(active_mux + 1, ch, -1);
                            //Debugging
                            printf("LED index: %d", led_idx);
                            led_buffer[led_idx] = set_colour(RED, 1.0);
                            for (int l = 0; l < NUM_PIXELS; l++) {
                                pio_sm_put_blocking(pio, sm, led_buffer[l]);
                            }
                            sleep_ms(200);
                            indicate_note_colors(current_index, current_index + 1);  // Restore proper current display
                        }

                        active_mux = -1;  // Clear press flag
                    }
                }

                // Wait for release
                if (active_mux == -1) {
                    if (last_adc_zero_time == 0) {
                        last_adc_zero_time = to_ms_since_boot(get_absolute_time());
                    } else if (to_ms_since_boot(get_absolute_time()) - last_adc_zero_time >= 600 && note_detected) {
                        process_sensor_buffer();
                        last_adc_zero_time = 0;
                        current_index++;  // ✅ Only increment when correct press was confirmed
                        break;  // Exit MUX/channel loop
                    }
                } else {
                    last_adc_zero_time = 0;  // Still active, reset
                }
            }

            sleep_ms(5);  // Global debounce
        }
    }

    printf("🎵 Demo complete!\n");
}

void Test(){
    printf("Starting Test mode...\n");

    int current_index = 0;
    uint64_t last_adc_zero_time = 0;

    while (current_index < NUM_DEMO_NOTES) {
        NoteInstruction current_note = demo_notes[current_index];
        NoteInstruction next_note = (current_index + 1 < NUM_DEMO_NOTES) ? demo_notes[current_index + 1] : (NoteInstruction){-1, -1, ""};

        indicate_note_colors(current_index, current_index + 1);  // ✅ Show current only when ready

        bool note_detected = false;

        while (!note_detected) {
            for (int m = 0; m < 3; m++) {
                MuxConfig* current_mux = &mux_configs[m];
                adc_select_input(current_mux->adc_channel);

                for (int ch = 0; ch < MAX_CHANNELS; ch++) {
                    select_mux_channel(current_mux, ch);
                    sleep_ms(5);  // Debounce

                    if (active_mux != -1) {
                        printf("Detected press: MUX %d, CH %d\n", active_mux + 1, ch);
                        update_sensor_buffer(ch);

                        if ((active_mux + 1) == current_note.mux && ch == current_note.channel) {
                            printf("✅ Correct note %s pressed\n", current_note.note);
                            //Debugging:
                            printf("Active_mux: %d == Current mux: %d\n",active_mux+1,current_note.mux);
                            printf("Channel: %d == current channel: %d\n",ch,current_note.channel);
                            int led_idx = get_led_index(current_note.mux, current_note.channel, -1);
                            printf("LED index: %d", led_idx);
                            led_buffer[led_idx] = set_colour(BLUE, 1.0);
                            for (int l = 0; l < NUM_PIXELS; l++) {
                                pio_sm_put_blocking(pio, sm, led_buffer[l]);
                            }
                            sleep_ms(200);
                            clear_leds(pio, sm);
                            note_detected = true;

                        } else {
                            printf("❌ Incorrect press at MUX %d, CH %d\n", active_mux + 1, ch);
                            //Debugging
                            printf("Active_mux: %d == Current mux: %d\n",active_mux+1,current_note.mux);
                            printf("Channel: %d == current Mux: %d\n",ch,current_note.channel);
                            int led_idx = get_led_index(active_mux + 1, ch, -1);
                            //Debugging
                            printf("LED index: %d", led_idx);
                            led_buffer[led_idx] = set_colour(RED, 1.0);
                            for (int l = 0; l < NUM_PIXELS; l++) {
                                pio_sm_put_blocking(pio, sm, led_buffer[l]);
                            }
                            sleep_ms(200);
                            indicate_note_colors(current_index, current_index + 1);  // Restore proper current display
                        }

                        active_mux = -1;  // Clear press flag
                    }
                }

                // Wait for release
                if (active_mux == -1) {
                    if (last_adc_zero_time == 0) {
                        last_adc_zero_time = to_ms_since_boot(get_absolute_time());
                    } else if (to_ms_since_boot(get_absolute_time()) - last_adc_zero_time >= 600 && note_detected) {
                        process_sensor_buffer();
                        last_adc_zero_time = 0;
                        current_index++;  // ✅ Only increment when correct press was confirmed
                        break;  // Exit MUX/channel loop
                    }
                } else {
                    last_adc_zero_time = 0;  // Still active, reset
                }
            }

            sleep_ms(5);  // Global debounce
        }
    }
    printf("🎵 Demo complete!\n");
}
#pragma clientregion
//***************************************Client Stuff***************************************//
int get_index(int strip, int led){
    return ((strip - 1)*6) + led;  // Calculate the LED index based on strip and LED number
}
#pragma endregion

 //***************************************Other Stuff***************************************//
// todo get free sm
PIO pio;       // Global handle for the PIO (Programmable I/O) hardware block
uint sm;       // State machine ID for PIO
uint offset;   // Offset into the PIO program (not used in this snippet)

// Multi-threaded function to be run on Core 1 of the Raspberry Pi Pico
void core1_entry() {
    while (1) {
        if (chord_learn_state == LEARNING_CHORD && current_target_chord != NULL) {
            run_chord_learning_mode();
            chord_learn_state = WAITING_FOR_CHORD_COMMAND; // Reset for next round
            current_target_chord = NULL; // Clear after learning
        } else {
            // If not in learning mode, run other modes as needed:
            // Demo();
            Test();
        }
        sleep_ms(10); // Prevent tight loop
    }
}
int main() {
    stdio_init_all();  // Initialize all standard IO, including USB serial if connected
    while (!stdio_usb_connected) {
        sleep_ms(100);  // Polling every 100 ms
    }
    wifi_init(); //initialize wifi connection with blocking 

    udp_begin_receiving(); //begin udp receiving
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
        // led_index = rand() % ((NUM_PIXELS - 1) - 0 + 1) + 0;  // Random index from 0 to NUM_PIXELS-1
        // printf("LED INDEX: %d\n", led_index);

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
    // printf("starting\n");
    // led_index = get_index(rand() % 3 + 1, rand() % 6 + 1);  // Randomly select a LED index for the sequence for testing purposes, ideally we want to use get_index on json data

    // Wait until USB serial is connected (optional, useful for debugging)
    while(true){
        
    }
    // Clean up: unload the WS2812 program and free associated resources (unlikely to be reached)
    pio_remove_program_and_unclaim_sm(&ws2812_program, pio, sm, offset);
}
