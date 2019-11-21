#ifndef CRC_CHECK_H
#define CRC_CHECK_H
#include "convolutional.h"
#include "crc_generator.h"
#include "portable.h"



uint8_t crc_result_check(      const crc_polynomial_t poly,                      
                             uint8_t *msg_crc,
                    uint16_t msg_len,
                        uint8_t *msg,
                correct_convolutional *conv);

#endif

