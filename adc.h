/*
 * adc.h
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 *
 */

#ifndef ADC_H_
#define ADC_H_

#define ADC_INPUT_PIN P5_4

#define AVG_LENGTH (2)
#define MAX_READING_MV (330)
#define ADC_CALI_MAX 16500
#define ADC_CALI_MIN 0

inline void adc_init();
// void adc_record();
// void adc_report_avg();
// void adc_report_range();
inline void adc_start_conversion();
unsigned int adc_map_val(const unsigned int val);
inline void adc_clear_min_max();
void adc_set_calibration(unsigned int min, unsigned int max);
void adc_calibrate_on_range();
inline void adc_store_reading(unsigned int val);
void adc_log_reading();
unsigned int adc_get_avg();

// inline unsigned int adc_get_max_value();
// inline void adc_set_max_value(unsigned int val);
// inline unsigned int adc_get_min_value();
// inline void adc_set_min_value(unsigned int val);

#endif /* ADH_H_ */
