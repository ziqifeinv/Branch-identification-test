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
#include "stdlib.h"
#include "wq_debug.h"

#define PLC_SUPPORT_HW_TSFM 1

#if PLC_SUPPORT_HW_TSFM

/* define standard deviation control parameters for data */
#define WQ_VTB_COFF             1.8
#define WQ_VTB_DELTA_REF        20000
/* define the number of cycles of data processed each time */
#define WQ_VTB_PERIOD_NUM       25
/* define demodulation threshold, uint is 1us. */
#define WQ_VTB_DECODE_THRESHOLD 300
/* define max number of different bits when searching preamble */
#define WQ_VTB_PRE_NUM_MAX      5

/* define the preamble bit group size*/
#define WQ_VTB_PREM_BIT_GROUP_SIZE  3
#define WQ_VTB_TSFM_FRAME_LEN       7
/* define tsfm location information structure */
#define WQ_VTB_PREAMBLE_LENGTH      32
#define WQ_TSFM_PRM_SEARCH_RANGE    2

typedef struct _wq_tsfm_prm_bit_t {
    uint8_t bit_cnt;
    uint8_t bit_pos[WQ_VTB_PREM_BIT_GROUP_SIZE];
} wq_tsfm_prm_bit_t;

typedef struct _wq_tsfm_prm_info_t {
    uint32_t preamble;
    uint8_t pos;
    float smooth_pos;
    uint8_t err_bit;
    wq_tsfm_prm_bit_t bit_info[WQ_VTB_PREAMBLE_LENGTH];
} wq_tsfm_prm_info_t;

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
    /* preamble info */
    wq_tsfm_prm_info_t prm_info;
} rx_init_t;

uint8_t *wq_vtb_rx_create()
{
    uint8_t *ptr;
    rx_init_t *rx;

    ptr = malloc(sizeof(rx_init_t));
    if (ptr == NULL) {
        goto out;
    }
    memset(ptr, 0, sizeof(rx_init_t));
    rx = (rx_init_t*)ptr;
    rx->conv = correct_convolutional_create(2, 7,
        correct_conv_r12_7_polynomial);
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

uint8_t isInArray(uint8_t* array, uint8_t targetData, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (array[i] == targetData)
            return 1;
    }
    return 0;
}

uint8_t checkJump(uint16_t* data, uint8_t targetPos, float ave_delta) {
    if (!targetPos) {
        if (data[targetPos + 1] > ave_delta
            && data[targetPos] < ave_delta)
            return 1;
        else
            return 0;
    }

    if (targetPos == WQ_VTB_PERIOD_NUM - 1) {
        if (data[targetPos - 1] < ave_delta
            && data[targetPos] > ave_delta)
            return 1;
        else
            return 0;
    }

    if ((data[targetPos - 1] < ave_delta
        && data[targetPos] > ave_delta)
        || (data[targetPos + 1] > ave_delta
            && data[targetPos] < ave_delta))
        return 1;
    return 0;
}

uint8_t isExistNeighbor(uint16_t* pos_vec, uint8_t len, uint8_t targetPos) {
    for (uint8_t i = 0; i < len; i++) {
        if (!targetPos) {
            if (pos_vec[i] == 1)
                return 1;
        }

        if (targetPos == len - 1) {
            if (pos_vec[i] == len - 2)
                return 1;
        }

        if ((pos_vec[i] - targetPos == 1) || (targetPos - pos_vec[i] == 1))
            return 1;
    }
    return 0;
}

uint8_t dis_detect(uint16_t *delta_vec, int8_t *offset){

    //wq_tsfm_dbg_printf("enter detection function\n");
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
    wq_tsfm_dbg_printf("ave:%d, ave1:%d, var:%d \n", (int)ave, (int)ave1, (int)var);

    uint8_t up_num = 0, low_num = 0;
    uint8_t up_pos[WQ_VTB_PERIOD_NUM], low_pos[WQ_VTB_PERIOD_NUM];

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

    wq_tsfm_dbg_printf("up_num:%d, low_num:%d \n", up_num, low_num);

    if (!(up_num + low_num) || (up_num + low_num >= 4)) {
        *offset = 0;
        wq_tsfm_dbg_printf("return 0 from case 2");
        return 0;
    }
    else if (up_num + low_num == 1) {
        *offset = 0;
        uint8_t pos = up_num ? up_pos[up_num-1] : low_pos[low_num-1];
        if((pos <= 6) ||(pos >= 20)){
            *offset = 12 - pos;
        }
        wq_tsfm_dbg_printf("return 1 from case 1");
        return 1;
    } else {
        *offset = 0;
        uint16_t ave_pos = 0, count = 0, sum_pos = 0;
        //uint16_t temp[up_num + low_num];
        uint16_t* temp = malloc(sizeof(uint16_t) * (up_num + low_num));

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
        free(temp);
        wq_tsfm_dbg_printf("return 1 from case 2");
        return 1;
    }
}

uint8_t dis_detect_1(uint16_t* delta_vec, int8_t expect_pos,
    wq_tsfm_prm_bit_t* preamble_bits_data)
{
    //wq_tsfm_dbg_printf("enter detection function\n");
    uint32_t sum = 0;
    uint8_t num = 0;
    float var = 0.0, ave = 0.0, ave1 = 0.0;

    //initialized the output struct
    preamble_bits_data->bit_cnt = 0;
    memset(&preamble_bits_data->bit_pos[0], 0,
        sizeof(uint8_t) * WQ_VTB_PREM_BIT_GROUP_SIZE);

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        sum += delta_vec[i];
    }
    ave = sum / WQ_VTB_PERIOD_NUM;

    // modify the very abnormal data
    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        if (delta_vec[i] < 0.5 * ave || delta_vec[i] > 1.5 * ave
            || delta_vec[i] < 18000 || delta_vec[i] > 22000) {
            sum -= delta_vec[i];
            num++;
        }
    }
    ave1 = sum / (WQ_VTB_PERIOD_NUM - num);

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        if (delta_vec[i] < 0.5 * ave || delta_vec[i] > 1.5 * ave
            || delta_vec[i] < 18000 || delta_vec[i] > 22000) {
            delta_vec[i] = ave1;
        }
    }

    float temp_dis = 0.0;

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        float dis = ave1 - delta_vec[i];
        temp_dis += dis * dis;
    }
    var = math_sqrt_newton_iterative_tsfm(temp_dis / (WQ_VTB_PERIOD_NUM - 1));

    uint8_t pos_num = 0;
    uint8_t temp_pos[WQ_VTB_PERIOD_NUM] = { 0 };
    uint16_t min_val = 30000, max_val = 0;
    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        if (delta_vec[i] >= max_val) {
            max_val = delta_vec[i];
        }
        if (delta_vec[i] <= min_val) {
            min_val = delta_vec[i];
        }
    }

    if (var <= 5 || (max_val - min_val <= 20)) {
        wq_tsfm_dbg_printf("return 0 case 1 \n");
        return 0;
    }

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        if (delta_vec[i] > ave1 + WQ_VTB_COFF * var || delta_vec[i] < ave1 - WQ_VTB_COFF * var) {
            temp_pos[pos_num] = i + 1;
            pos_num++;
        }
    }

    if (!pos_num) {
        wq_tsfm_dbg_printf("return 0 case 2 \n");
        return 0;
    }

    uint8_t pos_num1 = 0, neighborNum = 0;
    //uint8_t neighborPos[pos_num], temp_pos_new[pos_num];
    uint8_t* neighborPos = malloc(sizeof(uint8_t) * pos_num);
    uint8_t* temp_pos_new = malloc(sizeof(uint8_t) * pos_num);
    memset(neighborPos, 0, sizeof(*neighborPos));
    memset(temp_pos_new, 0, sizeof(*temp_pos_new));

    for (uint8_t j = 0; j < pos_num; j++) {
        if (!(isInArray(neighborPos, temp_pos[j], pos_num))) {
            if (delta_vec[temp_pos[j] - 1] <= ave1) {
                if ((temp_pos[j] == WQ_VTB_PERIOD_NUM) && (delta_vec[temp_pos[j] - 1] <= ave1 - var)) {
                    temp_pos_new[pos_num1] = temp_pos[j];
                    pos_num1++;
                }
                else {
                    if (isInArray(temp_pos, temp_pos[j] + 1, WQ_VTB_PERIOD_NUM) && delta_vec[temp_pos[j]] > ave1 + var) {
                        temp_pos_new[pos_num1] = temp_pos[j];
                        pos_num1++;
                        neighborPos[neighborNum] = temp_pos[j] + 1;
                        neighborNum++;
                    }
                    if (!isInArray(temp_pos, temp_pos[j] + 1, WQ_VTB_PERIOD_NUM) && delta_vec[temp_pos[j]] > ave1 + var) {
                        temp_pos_new[pos_num1] = temp_pos[j];
                        pos_num1++;
                    }
                }
            }
            else {
                if ((temp_pos[j] == 1) && (delta_vec[temp_pos[j] - 1] >= ave1 + var)) {
                    temp_pos_new[pos_num1] = temp_pos[j];
                    pos_num1++;
                }
                else {
                    if (isInArray(temp_pos, temp_pos[j] - 1, WQ_VTB_PERIOD_NUM) && delta_vec[temp_pos[j] - 2] < ave1 - var) {
                        temp_pos_new[pos_num1] = temp_pos[j] - 1;
                        pos_num1++;
                        neighborPos[neighborNum] = temp_pos[j];
                        neighborNum++;
                    }
                    if (!isInArray(temp_pos, temp_pos[j] - 1, WQ_VTB_PERIOD_NUM) && delta_vec[temp_pos[j] - 2] < ave1 - var) {
                        temp_pos_new[pos_num1] = temp_pos[j] - 1;
                        pos_num1++;
                    }
                }
            }
        }
    }

    if (!pos_num1) {
        free(temp_pos_new);
        free(neighborPos);
        wq_tsfm_dbg_printf("return 0 case 3 \n");
        return 0;
    }

    uint8_t eff_num = min(pos_num1, WQ_VTB_PREM_BIT_GROUP_SIZE);
    preamble_bits_data->bit_cnt = eff_num;
    memcpy(&preamble_bits_data->bit_pos[0], temp_pos_new, sizeof(uint8_t) * eff_num);

    if (expect_pos < 0) { // during preamble search period
        wq_tsfm_dbg_printf("return 0 case 4 \n");
        free(temp_pos_new);
        free(neighborPos);
        return 0;
    }
    else {// during data demod period
        for (uint8_t i = 0; i < pos_num1; i++) {
            if (abs(temp_pos_new[i] - expect_pos) <= 2) {
                wq_tsfm_dbg_printf("return 1 case 1 \n");
                free(temp_pos_new);
                free(neighborPos);
                return 1;
            }
        }
        wq_tsfm_dbg_printf("return 0 case 5 \n");
        free(temp_pos_new);
        free(neighborPos);
        return 0;
    }
}

void wq_tsfm_prm_push_bit(rx_init_t* rx, wq_tsfm_prm_bit_t* bit_info)
{
    wq_tsfm_prm_bit_t* ptr = &rx->prm_info.bit_info[0];
    memcpy(ptr, ptr + 1,
        sizeof(wq_tsfm_prm_bit_t) * (WQ_VTB_PREAMBLE_LENGTH - 1));
    memcpy(ptr + (WQ_VTB_PREAMBLE_LENGTH - 1), bit_info,
        sizeof(wq_tsfm_prm_bit_t));
    return;
}

void wq_tsfm_seek_preamble(rx_init_t* rx, uint8_t head, uint8_t tail,
    unsigned long* actual_prm)
{
    uint8_t search_tail = tail;
    uint8_t search_head = head;
    wq_tsfm_prm_bit_t* bit_ptr = NULL;
    uint8_t i = 0, j = 0;
    rx->prm_info.err_bit = 0xff;

    while (search_head < search_tail) {
        uint32_t preamble = 0;
        uint8_t pos = 0;
        uint8_t err_bit[7] = { 0 };
        for (i = 0; i < WQ_VTB_PREAMBLE_LENGTH; i++) {
            bit_ptr = &rx->prm_info.bit_info[i];
            for (j = 0; j < bit_ptr->bit_cnt; j++) {
                if (bit_ptr->bit_pos[j] > search_head
                    && bit_ptr->bit_pos[j] <= search_tail) {
                    preamble |= 1 << i;
                    pos = bit_ptr->bit_pos[j];
                    break;
                }
            }
        }

        for (i = 0; i < 7; i++) {
            err_bit[i] = count_num(preamble ^ actual_prm[i]);
            if (err_bit[i] < rx->prm_info.err_bit) {
                rx->prm_info.err_bit = err_bit[i];
                if (rx->prm_info.err_bit <= rx->popcnt_threshold) {
                    rx->prm_info.pos = pos;
                    rx->prm_info.preamble = preamble;
                    wq_tsfm_dbg_printf("tsfm preamble found, preamble:%08x, data_len:%d, "
                        "pos:%d, err bit:%d", rx->prm_info.preamble, i + 1,
                        rx->prm_info.pos, rx->prm_info.err_bit);
                    printf("tsfm preamble found, preamble:%08x, data_len:%d, "
                        "pos:%d, err bit:%d \n", rx->prm_info.preamble, i + 1,
                        rx->prm_info.pos, rx->prm_info.err_bit);
                    rx->rec_msg_len = rx->conv->rate * (i + 1 + 3);
                    rx->packet_det_flag = 1;
                }
            }
        }

        search_head = (search_head >= WQ_TSFM_PRM_SEARCH_RANGE)
            ? (search_head - WQ_TSFM_PRM_SEARCH_RANGE) : 0;
        search_tail = (search_tail >= WQ_TSFM_PRM_SEARCH_RANGE)
            ? (search_tail - WQ_TSFM_PRM_SEARCH_RANGE) : 0;
    }
}

void wq_tsfm_decode_preamble(rx_init_t* rx, unsigned long* actual_prm)
{
    uint8_t find_pos = 0xff;

    if (rx->receive_bit_num < WQ_VTB_PREAMBLE_LENGTH) {
        return;
    }
    rx->prm_info.err_bit = 0xff;
    rx->prm_info.pos = 0xff;

    uint8_t search_tail = WQ_VTB_PERIOD_NUM;
    uint8_t search_head = WQ_VTB_PERIOD_NUM - WQ_TSFM_PRM_SEARCH_RANGE;
    wq_tsfm_seek_preamble(rx, search_head, search_tail, actual_prm);

    search_tail = WQ_VTB_PERIOD_NUM + 1;
    search_head = WQ_VTB_PERIOD_NUM - WQ_TSFM_PRM_SEARCH_RANGE + 1;
    wq_tsfm_seek_preamble(rx, search_head, search_tail, actual_prm);

    if (rx->prm_info.pos != 0xff) {
        find_pos = rx->prm_info.pos;
    }
}

void wq_tsfm_delta_buf_update(rx_init_t* rx, uint8_t offset)
{
    int8_t offset_temp = 0;

    if (offset != 0) {
        offset_temp = 12 - offset;
    }

    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM + offset_temp; i++) {
        rx->receive_delta_buf[i] =
            rx->receive_delta_buf[WQ_VTB_PERIOD_NUM + i - offset_temp];
    }
    rx->receive_delta_num = WQ_VTB_PERIOD_NUM + offset_temp;

    return;
}

uint8_t wq_vtb_bit_rec(uint8_t *r_handle, uint16_t delta,
    uint8_t *decoded_data, uint8_t *decoded_data_len)
{
    uint8_t decode_flag = 0, mask = 0, bit_index_in_byte = 0;
    uint8_t decode_msg_len_with_crc = 0, mask1 = 0;
    uint8_t min_val = 100, min_pos = 0;
    uint16_t diff[7] = { 0 };
    uint32_t byte_index = 0, j = 0;
    uint32_t possible_preamble = 0;
    unsigned long *preamble_vector = NULL;
    rx_init_t *rx = (rx_init_t *)r_handle;
    uint16_t delta_data[WQ_VTB_PERIOD_NUM] = { 0 };
    int8_t offset = 0;
    int8_t expect_pos = 0;
    wq_tsfm_prm_bit_t bit_info = { 0 };
    // save the input delta value each time
    rx->receive_delta_buf[rx->receive_delta_num] = delta;
    rx->receive_delta_num++;

    // if it is the first time for calling this fucntion, return directly.
    if (rx->receive_delta_num < 2 * WQ_VTB_PERIOD_NUM) {
        goto out;
    }
    iot_wq_printf("receive bit num:%d, delta data:", rx->receive_bit_num);
    for (uint8_t i = 0; i < WQ_VTB_PERIOD_NUM; i++) {
        delta_data[i] = rx->receive_delta_buf[i];
        iot_wq_printf("%d ", delta_data[i]);
    }
    iot_wq_printf("\n");

    //detect the disturbrance
    if (!rx->packet_det_flag) {
        expect_pos = -1;
    }
    else {
        //expect_pos = 12;
        expect_pos = rx->prm_info.pos;
    }

    mask = dis_detect_1(delta_data, expect_pos, &bit_info);
    wq_tsfm_dbg_printf("mask:%d, expect_pos:%d, cnt:%d, pos:%d, %d, %d ", mask, expect_pos, bit_info.bit_cnt,
        bit_info.bit_pos[0], bit_info.bit_pos[1], bit_info.bit_pos[2]);
    rx->receive_bit_num++;
    if (!rx->packet_det_flag) {
        wq_tsfm_prm_push_bit(rx, &bit_info);
        preamble_vector = preuso_noise_sequence();
        wq_tsfm_decode_preamble(rx, preamble_vector);
        if (rx->prm_info.pos == 0xff) {
            wq_tsfm_delta_buf_update(rx, 0);
        } else {
            for (j = 0; j < 160; j++) {
                rx->rec_data_buf[j] = 0;
            }
            wq_tsfm_delta_buf_update(rx, rx->prm_info.pos);
            rx->prm_info.pos = 12;
            rx->prm_info.smooth_pos = 12.0;
        }
        goto out;
    }

    if (mask) {
        uint8_t diff_pos_min = 0xff, diff_pos = 0xff, cnt_temp = 0xff;
        for (uint8_t i = 0; i < bit_info.bit_cnt; i++) {
            diff_pos = abs(rx->prm_info.pos - bit_info.bit_pos[i]);
            if (diff_pos <= WQ_TSFM_PRM_SEARCH_RANGE) {
                if (diff_pos < diff_pos_min) {
                    diff_pos_min = diff_pos;
                    cnt_temp = i;
                }
            }
        }
        if (diff_pos_min != 0xff) {
            float actual_pos_f = (float)bit_info.bit_pos[cnt_temp];
            rx->prm_info.smooth_pos = actual_pos_f * 0.2
                + rx->prm_info.smooth_pos * 0.8;
            rx->prm_info.pos = (uint8_t)rx->prm_info.smooth_pos
                + ((rx->prm_info.smooth_pos - (int)rx->prm_info.smooth_pos) >= 0.5 ? 1 : 0);
        }
    }
    wq_tsfm_delta_buf_update(rx, 0);

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
    wq_tsfm_dbg_printf("CRC check %s !!", (decode_flag ? "success" : "failure"));
    printf("CRC check %s !! \n", (decode_flag ? "success" : "failure"));
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
    memset(&rx->prm_info, 0, sizeof(wq_tsfm_prm_info_t));
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
