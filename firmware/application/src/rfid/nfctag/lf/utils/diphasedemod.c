#include "diphasedemod.h"

void diphase_reset(diphase *d) {
    d->at_boundary = true; 
}

void diphase_feed(diphase *d, uint8_t interval, bool *bits, int8_t *bitlen) {
    uint8_t t = d->rp(interval);
    *bitlen = -1;

    if (t == 2) {
        d->at_boundary = true;
        return;
    }

    if (d->at_boundary) {
        if (t == 0) {
            // 0.5T = 0

            *bitlen = 1;
            bits[0] = 0;
            d->at_boundary = false; 
            
        } else if (t == 1) {
            // 1T = 0

            *bitlen = 1;
            bits[0] = 1;
            d->at_boundary = true;
        }
        
    } else {
        if (t == 0) {
            // 0.5T. bit 0 completed

            *bitlen = 0; 
            d->at_boundary = true; 
            
        } else {
            // PROTOCOL ERROR
            d->at_boundary = true;
        }
    }
}