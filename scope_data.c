/*
 * scope_data.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 *
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "adc.h"
#include "delay.h"
#include "led.h"
#include "scope_data.h"
#include "scope_term.h"
#include "timers.h"

volatile static uint8_t scope_mode = SCOPE_MODE_AC;

volatile static unsigned int dc_value = 0;

volatile static unsigned int ac_pkpk = 0;
volatile static unsigned int ac_dc_offset = 0;
volatile static unsigned int ac_true_rms = 0;
volatile static unsigned long ac_rms_sum = 0;
volatile static unsigned int ac_freq = 0;
volatile static unsigned int ac_period = 0;
volatile static unsigned int histogram[HISTOGRAM_SIZE] = {};
static unsigned int histogram_div = SCOPE_AC_HIST_DIV;
uint8_t histogram_units = 0;
volatile static unsigned int num_samples = 0;
volatile static unsigned int num_peaks = 0;

volatile static unsigned int max_val = 0;
volatile static unsigned int min_val = 16000;

volatile static unsigned int max_prev = 0;
volatile static unsigned int min_prev = 16000;

volatile static unsigned int peak_delta = 0;
volatile static unsigned int ac_pkpk_prev = 0;

// Mode selction
inline uint8_t scope_get_mode() {
    return scope_mode;
}

// DC Mode data
inline unsigned int scope_get_dc_value() {
    // mV from 0 to 300
    return adc_map_val(dc_value);
}

inline unsigned int scope_get_true_rms() {
    // mV from 0 to 300
    // if (ac_rms_sum != 0) {
    //     ac_true_rms = sqrt(ac_rms_sum / num_samples);
    // }
    return adc_map_val(ac_true_rms);
}

inline unsigned int scope_get_ac_pkpk() {
    // mV from 0 to 300
    return adc_map_val(ac_pkpk_prev);
}

// AC Mode data
inline unsigned int scope_get_ac_dc_offset() {
    // mV from 0 to 300
    return adc_map_val(ac_dc_offset);
}

inline unsigned int scope_get_ac_freq() {
    // Hz from 1 to 1000
    return ac_freq;
}

inline unsigned int scope_get_ac_period() {
    // ms from 1 to 1000
    return ac_period;
}

inline unsigned int scope_get_histogram(uint8_t i) {
    // mV from 0 to 300
    return adc_map_val(histogram[i]);
}

inline unsigned int scope_get_histogram_div() {
    // ms or us from 1 to 999
    // assert(histogram_div * 9 <= 1000)
    return histogram_div;
}

inline uint8_t scope_get_histogram_units() {
    // 0 for ms, 1 for s
    return histogram_units;
}

inline unsigned int scope_get_min() {
    // value from 0 to 16k
    return min_val;
}

inline unsigned int scope_get_max() {
    // value from 0 to 16k
    return max_val;
}

// Number of samples taken since last term refresh
inline unsigned int scope_get_num_samples() {
    return num_samples;
}

inline void scope_reset_num_samples() {
    // ac_rms_sum = 0;
    num_samples = 0;
}

inline void scope_cycle_ac_data() {
    if (num_samples <= 60000) {
        return;
    }
    // Store peak data
    ac_pkpk_prev = ac_pkpk;
    peak_delta = ac_pkpk >> 2;

    // Calculate new peak to peak and rms
    ac_pkpk = max_val - min_val;
    ac_true_rms = sqrt((max_val * max_val) - (min_val * min_val));

    // Reset peak count
    ac_freq = num_peaks * FREQ_SCALING;  // >> 1;
    num_peaks = 0;

    // Store and reset min, max
    min_prev = min_val;
    max_prev = max_val;
    min_val = 16000;
    max_val = 0;
}

inline void count_peaks(unsigned int val) {
    static bool finding_peak = true;
    if (finding_peak) {
        // Finding peak
        if (val > max_prev - peak_delta) {
            num_peaks += 1;
            finding_peak = false;
        }
    } else {
        // Finding trough
        if (val < min_prev + peak_delta) {
            finding_peak = true;
        }
    }
}

inline void scope_update_histogram() {
    int i;
    for (i = HISTOGRAM_SIZE - 1; i > 0; i -= 1) {
        histogram[i] = histogram[i - 1];
    }
    switch (scope_mode) {
        case SCOPE_MODE_AC:
            histogram[0] = ac_true_rms;
            break;

        default:
            histogram[0] = dc_value;
            break;
    }
}

inline void scope_switch_mode() {
    int i;
    switch (scope_mode) {
        case SCOPE_MODE_DC:
            scope_mode = SCOPE_MODE_AC;
            histogram_div = SCOPE_AC_HIST_DIV;
            histogram_units = SCOPE_HIST_UNITS_S;
            timer_resume_fast();
            break;
        default:
            scope_mode = SCOPE_MODE_DC;
            histogram_div = SCOPE_DC_HIST_DIV;
            histogram_units = SCOPE_HIST_UNITS_MS;
            timer_stop_fast();
    }
    for (i = 0; i < HISTOGRAM_SIZE; i += 1) {
        histogram[i] = 0;
    }
}

// Process latest value from ADC
inline void scope_read_data() {
    int reduced_precision = 0;
    unsigned int avg_val = 0;
    // Read in new data
    adc_log_reading();
    num_samples += 1;

    // Get updated average
    avg_val = adc_get_avg();

    // Record new min or max values
    if (avg_val > max_val) {
        max_val = avg_val;
    } else if (avg_val < min_val) {
        min_val = avg_val;
    }

    // reduced_precision = avg_val >> 5;
    // ac_rms_sum += reduced_precision * reduced_precision;

    count_peaks(avg_val);
}

// Prep for screen refresh
inline void scope_refresh_data() {
    unsigned int avg_val = adc_get_avg();
    // Update histogram data
    scope_update_histogram();

    // AC/DC MODE
    dc_value = avg_val;

    // AC Mode
    if (scope_mode == SCOPE_MODE_AC) {
        // AC Mode
        // if (min_max_valid) {
        ac_dc_offset = (ac_pkpk >> 1) + min_val;
        //     dc_offset_valid =
        //         (min_val < ac_dc_offset) && (max_val > ac_dc_offset);
        // }
        ac_period = 1000 / ac_freq;
    }
}
