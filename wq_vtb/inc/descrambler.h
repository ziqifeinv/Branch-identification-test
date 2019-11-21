#ifndef DESCRAMBLER_H
#define DESCRAMBLER_H
#include "convolutional.h"


typedef uint16_t descramble_polynomial_t;


static const descramble_polynomial_t descramble_polynomial = 0x49;

void descramble(    const descramble_polynomial_t poly,
                    uint8_t *msg,
            uint16_t msg_len,
            uint8_t *descrambled_msg,
            correct_convolutional *conv);


#endif
