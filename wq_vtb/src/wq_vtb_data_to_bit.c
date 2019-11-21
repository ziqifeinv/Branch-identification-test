/****************************************************************************

Copyright(c) 2019 by WuQi Technologies. ALL RIGHTS RESERVED.

This Information is proprietary to WuQi Technologies and MAY NOT
be copied by any method or incorporated into another program without
the express written consent of WuQi. This Information or any portion
thereof remains the property of WuQi. The Information contained herein
is believed to be accurate and WuQi assumes no responsibility or
liability for its use in any way and conveys no license or title under
any patent or copyright and makes no representation or warranty that this
Information is free from patent or copyright infringement.

****************************************************************************/

#include "conv_common.h"
#include "crc_generator.h"
#include "scramble.h"
#include "interleaver.h"
#include "preamble_generator.h"

#if PLC_SUPPORT_HW_TSFM

#define MAX_MSG_LEN 7

/* transmitter struct that contains convolutional encoding
 * struct and a temporary buffer.
 */
typedef struct {
    correct_convolutional *conv;
    uint8_t tx_temp_buf[20];
} tx_init_t;

uint8_t *wq_vtb_tx_create(void)
{
    uint8_t *ptr;
    tx_init_t *tx;
    ptr = malloc( sizeof(tx_init_t));
    if (ptr == NULL) {
        goto out;
    }
    tx = (tx_init_t*)ptr;
    tx->conv = correct_convolutional_create(2, 7,
        correct_conv_r12_7_polynomial);
    for (uint16_t i = 0; i < 20; i++) {
        tx->tx_temp_buf[i] = 0;
    }
out:
    return ptr;
}

uint8_t wq_vtb_data_encode(uint8_t *t_handle, uint8_t *buf,
    uint8_t data_len)
{
    uint8_t enclen, enclen_bytes, coded_data_len = 0;
    tx_init_t *tx = (tx_init_t*)t_handle;

    // if input data length is greater than 7 bytes, return with an error.
    if(data_len > MAX_MSG_LEN){
        //printf("transmit data length is too large, please check!\n");
        goto out;
    }

    enclen = correct_convolutional_encode_len(tx->conv, data_len + 2);
    enclen_bytes = (enclen % 8) ? (enclen/8 + 1) : enclen/8;
    coded_data_len = enclen_bytes + 4; // prepend 4 bytes preamble

    crc_calculation(crc16_polynomial, buf, data_len, tx->tx_temp_buf,
        tx->conv);

    //for(uint16_t j = 0; j < data_len + 2; j++){
    //    printf("bytes with crc: %4x\n", tx->tx_temp_buf[j]);
    //}

    scramble(scramble_polynomial, tx->tx_temp_buf, data_len + 2, buf,
       tx->conv);

    //for(uint16_t j = 0; j < MAX_MSG_LEN + 2; j++){
    //    printf("scrambled bytes are: %4x\n", buf[j]);
    //}

    correct_convolutional_encode(tx->conv, buf, data_len + 2,
        tx->tx_temp_buf);

    //for(uint16_t j = 0; j < enclen_bytes; j++){
    //    printf("encoded bytes are: %4x\n", tx->tx_temp_buf[j]);
    //}

    interleave(tx->tx_temp_buf, enclen_bytes, buf);

    //for(uint16_t j = 0; j < enclen_bytes; j++){
    //   printf("interleaved encoded bytes are: %4x\n", buf[j]);
    //}

    preamble_prepend(buf, enclen_bytes);

    //for(uint16_t j = 0; j < coded_data_len; j++){
    //   printf("encoded bytes with preamble are: %4x\n", buf[j]);
    //}
out:
    return coded_data_len;
}

uint8_t wq_vtb_get_encode_data_len(uint8_t data_len)
{
    return PREAMBLE_BYTES + 2 * (data_len + 1 + 16/8);
}

void wq_vtb_tx_destroy(uint8_t *t_handle)
{
    tx_init_t *tx = (tx_init_t*)t_handle;

    correct_convolutional_destroy(tx->conv);
    free(tx);
}

#endif /* PLC_SUPPORT_HW_TSFM */
