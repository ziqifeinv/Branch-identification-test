#ifndef INTERLEAVER_H
#define INTERLEAVER_H
#include "convolutional.h"

#define INTERLEAVER_FEPTH 16



typedef uint16_t interleaver_matrix_t;


//const interleaver_matrix_t interleaver_matrix[][4] = {{1,2,3,4},{5,6,7,8},{9,10,11,12},{13,14,15,16}};

void interleave(    uint8_t *msg,
                      uint16_t msg_len,
                        uint8_t *interleaved_msg);


#endif

