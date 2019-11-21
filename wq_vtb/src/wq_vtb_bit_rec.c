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
#include "descrambler.h"
#include "syncronization.h"
#include "deinterleaver.h"
#include "crc_check.h"
#include "preamble_generator.h"
#include "wq_vtb_bit_rec.h"

#if PLC_SUPPORT_HW_TSFM

/* define standard deviation control parameters for data */
#define WQ_VTB_COFF             2.23
#define WQ_VTB_VAR_MAX          40
/* define the number of cycles of data processed each time */
#define WQ_VTB_PERIOD_NUM       25
/* define demodulation threshold, uint is 1us. */
#define WQ_VTB_DECODE_THRESHOLD 300
/* define max number of different bits when searching preamble */
#define WQ_VTB_PRE_NUM_MAX      5

/* receiver struct definition */
typedef struct {
    /* convolutional struct. */
    correct_convolutional *conv;
    /* a buffer to store two consecutive delta */
    uint16_t receive_delta_buf[50];
    /* a coach buffer with max 160 bytes */
    uint8_t rec_data_buf[160];
    /* length of data in buffer */
    uint8_t rec_msg_len;
    /* flag to indicate preamble has been found or not */
    uint8_t packet_det_flag;
    /* input bit number within one successful CRC period */
    uint32_t receive_bit_num;
    /* input delta number within one successful CRC period */
    uint32_t receive_delta_num;
    /* number of attempts to find preamble within one successful
     * CRC period
     */
    uint32_t preamble_try_count;
    /* a counter to control the avaiable message after detect the
     * preamble
     */
    uint32_t msg_count;
    /* number of correctly detect the preamble */
    uint16_t preamble_find_num;
    /* number of successfully crc check */
    uint16_t crc_success_num;
    /* preamble detection parameter configuration */
    uint8_t popcnt_threshold;
} rx_init_t;

uint8_t *wq_vtb_rx_create()
{
    uint8_t *ptr;
    rx_init_t *rx;

    ptr = malloc( sizeof(rx_init_t));
    if (ptr == NULL) {
        goto out;
    }
    rx = (rx_init_t*)ptr;
    rx->conv = correct_convolutional_create(2, 7,
        correct_conv_r12_7_polynomial);
    for (uint8_t i = 0; i < 2*WQ_VTB_PERIOD_NUM; i++) {
        rx->receive_delta_buf[i] = 0;
    }
    for (uint8_t i = 0; i < 160; i++) {
        rx->rec_data_buf[i] = 0;
    }

    rx->rec_msg_len = 0;
    rx->packet_det_flag = 0;
    rx->preamble_try_count = 0;
    rx->receive_delta_num = 0;
    rx->receive_bit_num = 0;
    rx->msg_count = 0;
    rx->preamble_find_num = 0;
    rx->crc_success_num = 0;
    rx->popcnt_threshold = WQ_VTB_PRE_NUM_MAX;
out:
    return ptr;
}

/**
 * @brief math_sqrt_newton_iterative() - Use newton-iterative to get square root.
 * @param x the input number that need calculate square root.
 * @return the square root of input number.
 *
 */
static float math_sqrt_newton_iterative_tsfm(float x)
{
    union {
        float x;
        int i;
    } u;
    u.x = x;
    u.i = 0x5f3759df - (u.i >> 1);
    return u.x * (1.5f - 0.5f * x * u.x * u.x) * x;
}

uint8_t dis_detect(uint16_t *delta_vec, int8_t *offset){

    //printf("enter detection function\n");
    uint32_t sum = 0; float ave = 0.0;
    uint8_t num = 0;
    float var = 0.0;

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        sum += delta_vec[i];
    }
    ave = sum/WQ_VTB_PERIOD_NUM;

    // modify the very abnormal data
    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        if(delta_vec[i] < 0.5*ave || delta_vec[i] > 1.5*ave){
            sum -= delta_vec[i];
            num++;
        }
    }
    float ave1 = sum/(WQ_VTB_PERIOD_NUM-num);

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        if(delta_vec[i] < 0.5*ave || delta_vec[i] > 1.5*ave){
            delta_vec[i] = ave1;
        }
    }

    float temp_dis = 0.0;

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        float dis = ave1 - delta_vec[i];
        temp_dis += dis * dis;
    }
    var = math_sqrt_newton_iterative_tsfm(temp_dis/(WQ_VTB_PERIOD_NUM -1));

    if (var < WQ_VTB_VAR_MAX) {
        *offset = 0;
        return 0;
    }

    uint8_t up_num = 0, low_num = 0;
    uint16_t up_pos[WQ_VTB_PERIOD_NUM], low_pos[WQ_VTB_PERIOD_NUM];

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++ ) {
        if (delta_vec[i] > ave1 + WQ_VTB_COFF * var) {
            up_pos[up_num] = i+1;
            up_num++;
        }
        if (delta_vec[i] < ave1 - WQ_VTB_COFF * var) {
            low_pos[low_num] = i+1;
            low_num++;
        }
    }

    if (!(up_num + low_num) || (up_num + low_num >= 4)) {
        *offset = 0;
        return 0;
    }
    else if (up_num + low_num == 1) {
        *offset = 0;
        uint8_t pos = up_num ? up_pos[up_num-1] : low_pos[low_num-1];
        if((pos <= 6) ||(pos >= 20)){
            *offset = 12 - pos;
        }
        return 1;
    } else {
        *offset = 0;
        uint16_t ave_pos = 0, temp[up_num + low_num], count = 0, sum_pos = 0;

        for (uint16_t i = 0; i < up_num + low_num; i++) {
            if (i < up_num) {
                temp[i]= up_pos[i];
                count++;
                continue;
            }
            temp[i]=low_pos[i-count];
        }
        for (uint16_t i = 0; i < up_num + low_num; i++) {
            sum_pos += temp[i];

        }
        ave_pos = sum_pos/(up_num + low_num);

        if ((ave_pos <= 6) ||(ave_pos >= 20)) {
            *offset = 12 - ave_pos;
        }
        return 1;
    }

}

uint8_t wq_vtb_bit_rec(uint8_t *r_handle, uint16_t delta,
    uint8_t *decoded_data, uint8_t *decoded_data_len)
{
    uint8_t decode_flag = 0, mask = 0, bit_index_in_byte;
    uint8_t decode_msg_len_with_crc, mask1;
    uint8_t min_val = 100, min_pos = 0;
    uint16_t diff[7];
    uint32_t byte_index, j;
    uint32_t possible_preamble;
    unsigned long *preamble_vector;
    rx_init_t *rx = (rx_init_t *)r_handle;
    uint16_t delta_data[WQ_VTB_PERIOD_NUM];
    int8_t offset;

    // save the input delta value each time
    rx->receive_delta_buf[rx->receive_delta_num] = delta;
    rx->receive_delta_num++;

    // if it is the first time for calling this fucntion, return directly.
    if (rx->receive_delta_num < 2 * WQ_VTB_PERIOD_NUM) {
        goto out;
    }

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        delta_data[i] = rx->receive_delta_buf[i];
    }

    //detect the disturbrance
    mask = dis_detect(delta_data, &offset);
    rx->receive_bit_num++;

    //adjust the buffer data based on offset
    for(uint8_t i = 0; i < WQ_VTB_PERIOD_NUM + offset;i++){
        rx->receive_delta_buf[i] =
            rx->receive_delta_buf[WQ_VTB_PERIOD_NUM + i - offset];
    }
    rx->receive_delta_num = WQ_VTB_PERIOD_NUM + offset;
    if (!rx->packet_det_flag) {
        byte_index = (rx->receive_bit_num - 1) / 8;
        bit_index_in_byte = (rx->receive_bit_num - 1) % 8;

        if (rx->receive_bit_num < PREAMBLE_LENGTH) {
            rx->rec_data_buf[byte_index] += mask << bit_index_in_byte;
            goto out;
        } else if (rx->receive_bit_num == PREAMBLE_LENGTH) {
            rx->rec_data_buf[byte_index] += mask << bit_index_in_byte;
        } else {
            update_preamble_register_right(rx->rec_data_buf, mask);
        }
        preamble_vector = preuso_noise_sequence();
        possible_preamble = get_potential_preamble(rx->rec_data_buf);
        for (j = 0; j < 7; j++) {
            diff[j] = count_num(possible_preamble ^ preamble_vector[j]);
            if (diff[j] < min_val) {
                min_val = diff[j];
                min_pos = (uint8_t)(j + 1);
            }
        }

        if (min_val <= rx->popcnt_threshold) {
            rx->rec_msg_len = rx->conv->rate * (min_pos + 3);
            rx->packet_det_flag = 1;
            rx->preamble_find_num++;
            if (rx->preamble_find_num > 60000) {
                rx->preamble_find_num = 0;
            }
            for (j = 0; j < 160; j++) {
                rx->rec_data_buf[j] = 0;
            }
            goto out;
        } else {
            rx->preamble_try_count++;
            goto out;
        }
    }
    rx->msg_count++;

    if (rx->packet_det_flag && rx->msg_count <= (rx->rec_msg_len * 8)) {
        byte_index = (rx->msg_count-1)/8,
        bit_index_in_byte = (rx->msg_count-1)%8;
        rx->rec_data_buf[byte_index] += mask << bit_index_in_byte;
        goto out;
    }

    deinterleave(rx->rec_data_buf, rx->rec_msg_len, decoded_data);
    decode_msg_len_with_crc = rx->rec_msg_len / 2 - 1;
    mask1 = 0x80;

    for (j = 0; j < rx->rec_msg_len * 8; j++) {
        rx->rec_data_buf[j] = (decoded_data[j / 8] & mask1) ? 255 : 0;
        mask1 >>= 1;
        if(mask1 == 0)
            mask1 = 0x80;
    }
    correct_convolutional_decode_soft(rx->conv, rx->rec_data_buf,
        rx->rec_msg_len * 8, decoded_data);
    descramble(descramble_polynomial, decoded_data, decode_msg_len_with_crc,
        rx->rec_data_buf, rx->conv);
    decoded_data_len[0] = decode_msg_len_with_crc - 2;

    decode_flag = crc_result_check(crc16_polynomial, rx->rec_data_buf,
        decode_msg_len_with_crc, decoded_data, rx->conv);
    if (decode_flag) {
        rx->crc_success_num++;
        if (rx->crc_success_num > 60000) {
            rx->crc_success_num = 0;
        }
    }
    for (uint8_t i = 0; i < 50; i++) {
        rx->receive_delta_buf[i] = 0;
    }
    for (uint16_t i = 0; i < 160; i++) {
        rx->rec_data_buf[i] = 0;
    }
    rx->msg_count = 0;
    rx->rec_msg_len = 0;
    rx->packet_det_flag = 0;
    rx->preamble_try_count = 0;
    rx->receive_delta_num = 0;
    rx->receive_bit_num = 0;
    correct_convolutional_destroy(rx->conv);
    rx->conv = correct_convolutional_create(2, 7,
        correct_conv_r12_7_polynomial);
out:
    return decode_flag;
}

void wq_vtb_rx_destroy(uint8_t *r_handle)
{
    rx_init_t *rx = (rx_init_t *)r_handle;

    correct_convolutional_destroy(rx->conv);
    free(rx);
}

#endif /* PLC_SUPPORT_HW_TSFM */
