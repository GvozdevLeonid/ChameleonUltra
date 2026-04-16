#include "diphasedemod.h"

void diphase_reset(diphase *d) {
    d->at_boundary = true; 
}

void diphase_feed(diphase *d, uint8_t interval, bool *bits, int8_t *bitlen) {
    uint8_t t = d->rp(interval);
    *bitlen = -1;

    if (t == 3) {
        d->at_boundary = true;
        return;
    }

    if (d->at_boundary) {
        if (t == 0) {
            // 1T
            *bitlen = 1;
            bits[0] = 0;
        } else if (t == 1) {
            // 1.5T
            *bitlen = 2;
            bits[0] = 1;
            bits[1] = 0;
            d->at_boundary = false;
        } else if (t == 2) {
            // 2T
            *bitlen = 2;
            bits[0] = 1;
            bits[1] = 1;
        }
    } else {
        if (t == 0) {
            // 1T
            *bitlen = 1;
            bits[0] = 0;
        } else if (t == 1) {
            // 1.5T
            *bitlen = 1;
            bits[0] = 1;
            d->at_boundary = true;
        } else {
            // 2T
            diphase_reset(d);
            return;
        }
    }
}