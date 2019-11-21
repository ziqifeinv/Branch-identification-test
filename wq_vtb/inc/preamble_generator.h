#ifndef PREAMBLE_GENERATOR_H
#define PREAMBLE_GENERATOR_H
#include "conv_common.h"

#define PREAMBLE_LENGTH 32
#define REGISTER_ORDER 9
#define PREAMBLE_BYTES PREAMBLE_LENGTH/8

#define PREAMBLE_TYPE_1_BYTE    0x1a1
#define PREAMBLE_TYPE_3_BYTE    0x1f2
#define PREAMBLE_TYPE_COUNT     2

void preamble_prepend(uint8_t *msg,
                         uint8_t msg_len);


unsigned long *preuso_noise_sequence(      );
unsigned long *preuso_noise_sequence_topo();

#endif

