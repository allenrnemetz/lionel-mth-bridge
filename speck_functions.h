/*
 * speck_functions.h
 * 
 * Speck encryption header for MTH WTIU communication
 * Based on the implementation from RTCRemote by Mark Divechhio
 * http://www.silogic.com/trains/RTC_Running.html
 * 
 * License: GNU General Public License v3.0
 */

#ifndef SPECK_FUNCTIONS_H
#define SPECK_FUNCTIONS_H

#include <Arduino.h>

// Speck encryption types and constants (from RTCRemote)
#define SPECK_TYPE uint16_t
#define SPECK_ROUNDS 22
#define SPECK_KEY_LEN 4

#define ROR(x, r) ((x >> r) | (x << ((sizeof(SPECK_TYPE) * 8) - r)))
#define ROL(x, r) ((x << r) | (x >> ((sizeof(SPECK_TYPE) * 8) - r)))
#define RRR(x, y, k) (x = ROR(x, 7), x += y, x ^= k, y = ROL(y, 2), y ^= x)

// Speck function prototypes
void speck_expand(SPECK_TYPE const K[SPECK_KEY_LEN], SPECK_TYPE S[SPECK_ROUNDS]);
void speck_encrypt(SPECK_TYPE const pt[2], SPECK_TYPE ct[2], SPECK_TYPE const K[SPECK_ROUNDS]);
void speck_encrypt_combined(SPECK_TYPE const pt[2], SPECK_TYPE ct[2], SPECK_TYPE const K[SPECK_KEY_LEN]);

#endif // SPECK_FUNCTIONS_H
