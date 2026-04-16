#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * 0 - 1T
 * 1 - 1.5T
 * 2 - 2T
 * 3 - error/noise (the interval is too long or too short)
 */
typedef uint8_t (*period)(uint8_t interval);

typedef struct {
    bool at_boundary;
    period rp;
} diphase;

extern void diphase_reset(diphase *d);
extern void diphase_feed(diphase *d, uint8_t interval, bool *bits, int8_t *bitlen);