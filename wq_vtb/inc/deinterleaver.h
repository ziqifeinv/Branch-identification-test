#ifndef DEINTERLEAVER_H
#define DEINTERLEAVER_H
#include <stdint.h>

#define INTERLEAVER_FEPTH 16

typedef uint16_t deinterleaver_matrix_t;



void deinterleave(uint8_t *msg,
                      uint16_t msg_len,
                        uint8_t *deinterleaved_msg);


#endif

