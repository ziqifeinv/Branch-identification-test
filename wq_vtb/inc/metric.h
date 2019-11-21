#ifndef METRIC_H
#define METRIC_H
#include "convolutional.h"
// measure the hamming distance of two bit strings
// implemented as population count of x XOR y
static inline distance_t metric_distance(uint16_t x, uint16_t y) {
    return count_num(x ^ y);
}

static inline distance_t metric_soft_distance_linear(uint16_t hard_x, const uint8_t *soft_y, size_t len) {
    distance_t dist = 0;
    for (uint16_t i = 0; i < len; i++) {
        uint16_t soft_x = ((int8_t)(0) - (hard_x & 1)) & 0xff;
        hard_x >>= 1;
        int d = soft_y[i] - soft_x;
        dist += (d < 0) ? -d : d;
    }
    return dist;
}

distance_t metric_soft_distance_quadratic(uint16_t hard_x, const uint8_t *soft_y, size_t len);
#endif