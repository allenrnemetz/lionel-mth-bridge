/*
 * speck_functions.cpp
 * 
 * Speck encryption implementation for MTH WTIU communication
 * Based on the implementation from RTCRemote by Mark Divechhio
 * http://www.silogic.com/trains/RTC_Running.html
 * 
 * License: GNU General Public License v3.0
 */

#include <Arduino.h>
#include "speck_functions.h"

// Speck encryption implementation
void speck_expand(SPECK_TYPE const K[SPECK_KEY_LEN], SPECK_TYPE S[SPECK_ROUNDS]) {
    SPECK_TYPE i, b = K[0];
    SPECK_TYPE a[SPECK_KEY_LEN - 1];
    
    for (i = 0; i < SPECK_KEY_LEN - 1; i++) {
        a[i] = K[i + 1];
    }
    
    for (i = 0; i < SPECK_ROUNDS; i++) {
        S[i] = b;
        RRR(b, a[i % (SPECK_KEY_LEN - 1)], i);
    }
}

void speck_encrypt(SPECK_TYPE const pt[2], SPECK_TYPE ct[2], SPECK_TYPE const K[SPECK_ROUNDS]) {
    SPECK_TYPE i;
    ct[0] = pt[0];
    ct[1] = pt[1];
    
    for (i = 0; i < SPECK_ROUNDS; i++) {
        ct[1] = ROL(ct[1], 2);
        ct[1] += ct[0];
        ct[1] ^= K[i];
        ct[0] = ROR(ct[0], 7);
        ct[0] ^= ct[1];
    }
}

void speck_encrypt_combined(SPECK_TYPE const pt[2], SPECK_TYPE ct[2], SPECK_TYPE const K[SPECK_KEY_LEN]) {
    SPECK_TYPE S[SPECK_ROUNDS];
    speck_expand(K, S);
    speck_encrypt(pt, ct, S);
}
