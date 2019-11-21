#ifndef CONV_DECODE_H
#define CONV_DECODE_H
#include "convolutional.h"


ssize_t correct_convolutional_decode_soft(correct_convolutional *conv, const soft_t *encoded,
                                          size_t num_encoded_bits, uint8_t *msg);


ssize_t correct_convolutional_decode(correct_convolutional *conv, const uint8_t *encoded,
                                     size_t num_encoded_bits, uint8_t *msg);



#endif

