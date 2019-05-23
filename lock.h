/*
 * lock.h
 *
 * Danica Fujiwara & Spencer Shaw
 *
 * CPE 329-17/18 Spring 2019
 */

#ifndef __LOCK_H__
#define __LOCK_H__

#include "stdint.h"

// Time a key must remain pressed to be read
#define LOCK_HOLD_MS (5)

// Data containing a passcode
typedef struct passcode_s {
    uint8_t dig1;
    uint8_t dig2;
    uint8_t dig3;
    uint8_t dig4;
} passcode_t;

// Execute the lock routine
uint8_t lock(const passcode_t passcode);

// Display the locked message
void lock_message();

// Display the unlocked message
void unlock_message();

// Check two passcodes for equality
uint8_t check_passcode(passcode_t actual, passcode_t guess);

// Construct a passcode
passcode_t passcode_init(uint8_t dig1,
                         uint8_t dig2,
                         uint8_t dig3,
                         uint8_t dig4);

// Construct and empty passcode
passcode_t passcode_empty();

#endif
