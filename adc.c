/*
 * adc.c
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 *
 */
#include "msp.h"

#include "adc.h"
#include "led.h"
#include "uart.h"

static unsigned int adc_value = 0;
static unsigned int calibration_shift = 0;
static unsigned int calibration_scale = 5;
static unsigned int adc_history[AVG_LENGTH]; // = {0, 0, 0, 0};
static unsigned int max_value = 0;
static unsigned int min_value = 16000;

inline void adc_init() {
    rgb_set(RGB_GREEN);

    // GPIO Setup
    P5->SEL1 |= ADC_INPUT_PIN;  // Configure P5.4 for ADC
    P5->SEL0 |= ADC_INPUT_PIN;

    P5->REN &= ~ADC_INPUT_PIN;

    // Sampling time, S&H=16, ADC14 on
    ADC14->CTL0 = ADC14_CTL0_SHT0_2 | ADC14_CTL0_SHP | ADC14_CTL0_ON;
    ADC14->CTL1 = ADC14_CTL1_RES_3;  // Use sampling timer, 14-bit conversion

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_1;  // A1 ADC input select; Vref=AVCC
    ADC14->IER0 |= ADC14_IER0_IE0;         // Enable ADC conv complete interrupt
    ADC14->CTL0 |= ADC14_CTL0_ENC;         // Enable conversions

    // Enable global interrupt
    __enable_irq();

    // Enable ADC interrupt in NVIC module
    NVIC->ISER[0] = 1 << ((ADC14_IRQn)&31);

    adc_set_calibration(ADC_CALI_MIN, ADC_CALI_MAX);
    
    adc_start_conversion();

    rgb_clear(RGB_GREEN);
}

// // Store last value and calculate avg
// void adc_record() {
//     unsigned int avg_val = 0;
//     //   unsigned int avg_val_mv = 0;

//     adc_log_reading();
//     avg_val = adc_get_avg();

//     if (avg_val > max_value) {
//         max_value = avg_val;
//     }
//     if (avg_val < min_value) {
//         min_value = avg_val;
//     }

//     adc_start_conversion();
// }

// void adc_report_avg() {
//     unsigned int avg_val = adc_get_avg();
//     unsigned int avg_val_mv = adc_map_val(avg_val);

//     uart_write_volts(avg_val_mv);
// }

// void adc_report_range() {
//     uart_write('m');
//     uart_write('i');
//     uart_write('n');
//     uart_write(':');
//     uart_write(' ');
//     uart_write_int(min_value);
//     uart_write_nl();
//     uart_write('m');
//     uart_write('a');
//     uart_write('x');
//     uart_write(':');
//     uart_write(' ');
//     uart_write_int(max_value);
//     uart_write_nl();
// }

// Start sampling/conversion
inline void adc_start_conversion() {
    ADC14->CTL0 |= ADC14_CTL0_SC;
}

unsigned int adc_map_val(const unsigned int val) {
    unsigned int val_mv = (val - calibration_shift) / calibration_scale;
    return val_mv;
}

void adc_set_calibration(unsigned int min, unsigned int max) {
    calibration_shift = min;
    calibration_scale = (max - min) / MAX_READING_MV;
    adc_clear_min_max();
}

inline void adc_clear_min_max() {
    min_value = 16000;
    max_value = 0;
}

void adc_calibrate_on_range() {
    adc_set_calibration(min_value, max_value);
}

// In ISR, write value from ADC to static variable
inline void adc_store_reading(unsigned int val) {
    adc_value = val;
}

// Log last value to avg array
void adc_log_reading() {
    int i;
    for (i = 0; i < AVG_LENGTH - 1; i++) {
        adc_history[i + 1] = adc_history[i];
    }
    adc_history[0] = adc_value;
}

// Calculate avgerage from array
unsigned int adc_get_avg() {
    int i;
    unsigned int sum = 0;
    for (i = 0; i < AVG_LENGTH; i++) {
        sum += adc_history[i];
    }
    return sum / AVG_LENGTH;
}

// inline unsigned int adc_get_max_value() {
//     return max_value;
// }
// inline void adc_set_max_value(unsigned int val) {
//     max_value = val;
// }

// inline unsigned int adc_get_min_value() {
//     return min_value;
// }

// inline void adc_set_min_value(unsigned int val) {
//     min_value = val;
// }
