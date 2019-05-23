/*
 * button.h
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 *
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdint.h>
#include <stdbool.h>

#define BUTTON_PIN P1_1
#define MANUAL_PIN P1_5

#define TRIGGER_PIN BUTTON_PIN

inline void button_init();
inline bool button_get();

#endif /* BUTTON_H_ */
