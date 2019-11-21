#ifndef SCRAMBLE_H
#define SCRAMBLE_H
#include "convolutional.h"

typedef uint16_t scramble_polynomial_t;


static const scramble_polynomial_t scramble_polynomial = 0x49;

void scramble(   const scramble_polynomial_t poly,
                    uint8_t *msg,
            uint8_t msg_len,
            uint8_t *scrambled_msg,
            correct_convolutional *conv);

#endif

