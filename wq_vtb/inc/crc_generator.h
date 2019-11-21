#ifndef CRC_GENERATOR_H
#define CRC_GENERATOR_H
#include "convolutional.h"



typedef uint16_t crc_shift_register_t;

typedef uint16_t crc_polynomial_t;

// define generator polynomial of CRC16, x^16+x^12+x^5+1.
static const crc_polynomial_t crc16_polynomial = 0x1021;


void crc_calculation(      const crc_polynomial_t poly,                      
                              uint8_t *msg,
                    uint16_t msg_len,
                        uint8_t *msg_crc,
                    correct_convolutional *conv);

#endif

