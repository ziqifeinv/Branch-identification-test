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
#include "disturbance_detection.h"
#include "wq_debug.h"
#include "wq_vtb_topo_rec.h"
#include <stdlib.h>
#include "wq_version.h"

#define PLC_SUPPORT_HW_TOPO 1
#if PLC_SUPPORT_HW_TOPO

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
/* define the delta data to be processed */
#define WQ_VTB_ZC_DELTA_DEAL_NUMBER   1000
/* define the delta circular array max size */
#define WQ_VTB_ZC_DELTA_BUF_SIZE_MAX  1500
/* maximum number of deltas per processing */
#define WQ_VTB_COLLECT_ZC_CNT_MAX     64
/* define delta maximum */
#define WQ_VTB_ZC_DELTA_VALUE_MAX     21000
/* define delta minimum */
#define WQ_VTB_ZC_DELTA_VALUE_MIN     19000

/* current pos weight
 * next_pos = curr_pos * 0.8 + last_pos * (1 - 0.8)
 */
#define WQ_VTB_POS_WERGHT             8
 /* defines the retrieval scope when the signals are merged */
#define WQ_VTB_WAVELET_PEAK_RANGE     (128 + 32)
/* define amplitude difference between signal and noise */
#define WQ_VTB_AMPLITUDE_THRESHOLD    800

typedef struct _wq_vtb_topo_dalta_t {
    /* define the ring buffer that holds the delta data */
    uint32_t delta_buf[WQ_VTB_ZC_DELTA_BUF_SIZE_MAX];
    /* define last updated position */
    uint16_t pos_last;
    /* define first updated position */
    uint16_t pos_first;
    /* the value is always the sum of the latest deltas.
     * the number of participants is IOT_VTB_DELTA_DEAL_NUMBER
     */
    uint32_t sum;
    /* the value is always the average of the latest deltas. */
    uint32_t average;
    /* 1 -> the WQ_VTB_ZC_DELTA_DEAL_NUMBER number of deltas
     * has been saved and processed
     */
    uint8_t ready;
    /* first delta data gp_timer0 counter,
     * only WQ_VTB_ZC_DELTA_DEAL_NUMBER numbers are counted
     */
    uint64_t timer_counter;
} wq_vtb_topo_delta_t;

/* receiver struct definition */
typedef struct {
    /* convolutional struct */
    correct_convolutional* conv;
    /* three buffers with max 160 bytes */
    uint8_t phase_data_buf[3][160];
    /* length of data in buffer */
    uint8_t rec_msg_len[3];
    /* flag to indicate preamble has been found or not */
    uint8_t packet_det_flag[3];
    /* number of received bits (from demodulation) */
    uint32_t receive_bit_num[3];
    /* a counter to control the avaiable message after detect preamble */
    uint32_t msg_count[3];
    /* number of attempts to find preamble */
    uint32_t preamble_try_count[3];
    /* time of receiver function to be called */
    uint32_t call_time[3];
    /* number of correctly detect preamble */
    uint16_t preamble_find_num[3];
    /* number of successful crc check */
    uint16_t crc_success_num[3];
    /* preamble detection parameter configuration */
    uint8_t popcnt_threshold;
    /* to store preamble bits. */
    wq_prm_bit_val_t preamble_bits[PREAMBLE_PHASE_NUM][PREAMBLE_BIT_LEN];
    /* correction for adc data. */
    int32_t correct_offset[PREAMBLE_PHASE_NUM];
    /* avg value for each phase. */
    float avg_value[PREAMBLE_PHASE_NUM];
    /* the distortion direction for each phase */
#if(TOPO_VERSION >= TOPO_V3_4)
    uint16_t sign_sum[PREAMBLE_PHASE_NUM];
    uint16_t bit1_counter[PREAMBLE_PHASE_NUM];
#else if(TOPO_VERSION >= TOPO_V3_3)
    uint16_t act_bits[PREAMBLE_PHASE_NUM];
    uint16_t neg_bits[PREAMBLE_PHASE_NUM];
#endif
    /* historical information of distortion position */
    uint16_t pos_smooth[PREAMBLE_PHASE_NUM];
    uint8_t first_mask[PREAMBLE_PHASE_NUM];
} rx_topo_init_t;

typedef struct _iot_rx_topo_bits_timming_t {
    uint32_t    pos;
    float       val;
    uint8_t     sign;
}iot_bits_timming_t;

typedef struct _iot_rx_topo_preamble_t {
    uint32_t    prmb;
    uint32_t    err_bits;
    uint32_t    pos;
    float       val;
}iot_rx_topo_preamble_t;

/* define data of delta struct */
static wq_vtb_topo_delta_t* g_wq_vtb_topo_delta = NULL;

#if WQ_TOPO_DETECT_DEBUG_PRINT
void wq_debug_dump_data(void* buf, uint32_t len, uint32_t elm_size)
{
    uint32_t i, * p_uint32 = buf;
    uint16_t* p_uint16 = buf;
    uint8_t* p_uint8 = buf;

    for (i = 0; i < len; i++) {
        if (0 == (i % 32)) {
            iot_wq_printf("\r\n");
        }
        if (sizeof(uint32_t) == elm_size) {
            iot_wq_printf("%u,", *p_uint32++);
        }
        else if (sizeof(uint16_t) == elm_size) {
            iot_wq_printf("%u,", *p_uint16++);
        }
        else {
            iot_wq_printf("%u,", *p_uint8++);
        }
    }

    return;
}
#define wq_debug_dump(p_buf, len) wq_debug_dump_data(p_buf, len, sizeof(*p_buf))
#else
#define wq_debug_dump(p_buf, len)
#endif /* WQ_TOPO_DETECT_DEBUG_PRINT */

uint8_t* wq_vtb_topo_rx_create(uint8_t pop_num)
{
    uint8_t* ptr;
    rx_topo_init_t* rx;
    ptr = malloc(sizeof(rx_topo_init_t));
    if (ptr == NULL) {
        goto out;
    }
    rx = (rx_topo_init_t*)ptr;
    memset(ptr, 0, sizeof(*rx));
    rx->conv = correct_convolutional_create(2, 7,
        correct_conv_r12_7_polynomial);
    if (!rx->conv) {
        free(ptr);
        ptr = NULL;
        goto out;
    }
    rx->popcnt_threshold = pop_num;

    for (uint8_t i = 0; i < PREAMBLE_PHASE_NUM; i++) {
        rx->first_mask[i] = 1;
    }
out:
    return ptr;
}

uint8_t wq_vtb_topo_zc_delta_handle(uint8_t* data, uint8_t data_len,
    uint32_t last_counter)
{
    wq_vtb_topo_delta_t* delta = NULL;
    uint32_t temp[WQ_VTB_COLLECT_ZC_CNT_MAX] = { 0 };
    uint8_t i = 0, temp_len = 0;
    uint32_t sum_old = 0, sum_new = 0;
    uint8_t byte_len = data_len * 4;
    uint16_t temp_diff = 0;

    memcpy(temp, data, byte_len);
    if (g_wq_vtb_topo_delta == NULL) {
        g_wq_vtb_topo_delta = malloc(sizeof(*g_wq_vtb_topo_delta));
        if (g_wq_vtb_topo_delta == NULL) {
            printf("%s mem malloc failed! \n", __FUNCTION__);
            assert(0);
        }
        memset(g_wq_vtb_topo_delta, 0, sizeof(wq_vtb_topo_delta_t));
        g_wq_vtb_topo_delta->timer_counter = last_counter;
        for (i = 0; i < data_len; i++) {
            g_wq_vtb_topo_delta->timer_counter -= temp[i];
        }
    }
    delta = g_wq_vtb_topo_delta;
    for (i = 0, sum_new = 0; i < data_len; i++) {
        if (temp[i] > WQ_VTB_ZC_DELTA_VALUE_MAX
            || temp[i] < WQ_VTB_ZC_DELTA_VALUE_MIN) {
            printf("[topo debug]%s zc delta abnormal:%d \n",
                __FUNCTION__, temp[i]);
        }
        sum_new += temp[i];
    }

    if ((delta->pos_last >= WQ_VTB_ZC_DELTA_DEAL_NUMBER)
        && delta->ready != 1) {
        delta->pos_first = delta->pos_last - WQ_VTB_ZC_DELTA_DEAL_NUMBER;
        delta->ready = 1;
    }

    if (delta->pos_last + data_len > WQ_VTB_ZC_DELTA_BUF_SIZE_MAX) {
        temp_len = WQ_VTB_ZC_DELTA_BUF_SIZE_MAX - delta->pos_last;
        memcpy(&delta->delta_buf[delta->pos_last], temp, temp_len * 4);
        memcpy(&delta->delta_buf[0], &temp[temp_len],
            (data_len - temp_len) * 4);
        delta->pos_last = (data_len - temp_len);
    }
    else {
        memcpy(&delta->delta_buf[delta->pos_last], temp, byte_len);
        delta->pos_last += data_len;
        if (delta->pos_last >= WQ_VTB_ZC_DELTA_BUF_SIZE_MAX) {
            delta->pos_last = 0;
        }
    }

    delta->sum += sum_new;
    if (delta->ready != 1) {
        return 0;
    }
    if (delta->pos_first + data_len >= WQ_VTB_ZC_DELTA_BUF_SIZE_MAX) {
        sum_old = 0;
        temp_len = WQ_VTB_ZC_DELTA_BUF_SIZE_MAX - delta->pos_first;
        for (i = 0; i < temp_len; i++) {
            sum_old += delta->delta_buf[delta->pos_first + i];
        }
        delta->pos_first = 0;
        temp_len = data_len - temp_len;
        for (i = 0; i < temp_len; i++) {
            sum_old += delta->delta_buf[delta->pos_first + i];
        }
        delta->pos_first = temp_len;
    }
    else {
        for (i = 0, sum_old = 0; i < data_len; i++) {
            sum_old += delta->delta_buf[delta->pos_first + i];
        }
        delta->pos_first += data_len;
    }
    delta->sum -= sum_old;
    delta->timer_counter += sum_old;
    delta->average = delta->sum / WQ_VTB_ZC_DELTA_DEAL_NUMBER;

    if (delta->pos_last > delta->pos_first) {
        temp_diff = delta->pos_last - delta->pos_first;
    }
    else {
        temp_diff = WQ_VTB_ZC_DELTA_BUF_SIZE_MAX - delta->pos_first
            + delta->pos_last;
    }
    assert(temp_diff == WQ_VTB_ZC_DELTA_DEAL_NUMBER);
    return 0;
}

void wq_preamble_push_bit(rx_topo_init_t* rx, uint32_t phase,
    wq_prm_bit_val_t* preamble_bit)
{
    wq_prm_bit_val_t* p_prm = &rx->preamble_bits[phase][0];

    /* Push one buffer ahead */
    memcpy(p_prm, p_prm + 1, sizeof(*p_prm) * (PREAMBLE_BIT_LEN - 1));
    /* Store coming buffer on the tail. */
    memcpy(p_prm + (PREAMBLE_BIT_LEN - 1), preamble_bit, sizeof(*p_prm));

    return;
}

void wq_preamble_clear(rx_topo_init_t* rx, uint32_t phase)
{
    wq_prm_bit_val_t* p_prm = &rx->preamble_bits[phase][0];

    memset(p_prm, 0x0, sizeof(*p_prm) * PREAMBLE_BIT_LEN);

    return;
}

uint32_t wq_preamble_copy(rx_topo_init_t* rx, uint32_t phase, uint32_t* buf,
    uint8_t* timing_mag, float* timing_val, uint32_t buf_len, uint32_t scale)
{
    uint32_t g_idx, b_idx, bits_cnt = 0, detla_scale;
    wq_prm_bit_val_t* p_prm = NULL;

    for (b_idx = 0; b_idx < PREAMBLE_BIT_LEN; b_idx++) {
        p_prm = &rx->preamble_bits[phase][b_idx];
        detla_scale = b_idx * scale;
        for (g_idx = 0; (g_idx < p_prm->bit_cnt)
            && (g_idx < PREAMBLE_BIT_GROUP); g_idx++) {
            buf[bits_cnt] = p_prm->bit_pos[g_idx] + detla_scale;
            timing_mag[bits_cnt] = p_prm->mag_sign[g_idx];
            timing_val[bits_cnt] = p_prm->mag_val[g_idx];
            bits_cnt++;
            if (bits_cnt >= buf_len) {
                goto copy_out;
            }
        }
    }

copy_out:

    return bits_cnt;
}

uint32_t wq_preamble_scan_rough(rx_topo_init_t* rx, uint32_t phase,
    uint32_t scale, uint32_t delta, wq_prm_val_t* preamble, uint32_t preamble_cnt)
{
    uint32_t timing_pos[PREAMBLE_BIT_LEN * PREAMBLE_BIT_GROUP];
    uint8_t timing_mag[PREAMBLE_BIT_LEN * PREAMBLE_BIT_GROUP];
    float timing_val[PREAMBLE_BIT_LEN * PREAMBLE_BIT_GROUP];
    uint32_t pre_pos, pst_pos, cur_pos, preamble_bits, bit, end_pos, preamble_pos;
    uint32_t preamble_idx = 0, last_preamble_bits = 0, pos_cnt = 0;
    int32_t i;

    memset(timing_pos, 0, sizeof(timing_pos));
    memset(timing_mag, 0, sizeof(timing_mag));
    memset(timing_val, 0, sizeof(timing_val));

    pos_cnt = wq_preamble_copy(rx, phase, timing_pos, timing_mag, timing_val,
        sizeof(timing_pos) / sizeof(timing_pos[0]), scale);

    if (pos_cnt == 0) {
        return 0;
    }

    /* when the signal satisfies the following conditions, they need to be merged
     * 1.around a multiple of 3205(scale)
     * 2.the position difference between the two signals is less than 128+32
     * 3.the amplitude difference between the two signals is less than 800
     */
    for (i = pos_cnt - 1; i > 0; i--) {
        if ((timing_pos[i] % scale < WQ_VTB_WAVELET_PEAK_RANGE)
            && (timing_pos[i - 1] % scale > scale - WQ_VTB_WAVELET_PEAK_RANGE)
            && (timing_pos[i] - timing_pos[i - 1] < WQ_VTB_WAVELET_PEAK_RANGE)
            && (abs(timing_val[i] - timing_val[i - 1])
                < WQ_VTB_AMPLITUDE_THRESHOLD)) {
            timing_pos[i] = timing_pos[i - 1];
            timing_mag[i] = timing_mag[i - 1];
        }
    }

    //wq_dbg_printf("phase %d timing count %d:", phase,pos_cnt );
    //wq_debug_dump(timing_pos, pos_cnt);

    cur_pos = PREAMBLE_BIT_LEN * scale - delta;
    end_pos = PREAMBLE_BIT_LEN * scale - scale;

    while (cur_pos >= end_pos) {
        uint32_t pos_next = 0, pos_last = 0;
        pre_pos = cur_pos - delta;
        pst_pos = cur_pos + delta;
        preamble_bits = 0;
        preamble_pos = 0;

        for (bit = PREAMBLE_LENGTH; bit > 0; bit--) {
            preamble_bits <<= 1;
            preamble[preamble_idx].bits_pos[bit - 1] = 0;
            preamble[preamble_idx].mag_sign[bit - 1] = 0;
            for (i = pos_cnt - 1; ((i >= 0)
                && (timing_pos[i] >= pre_pos)); i--) {
                if ((timing_pos[i] > pre_pos) && (timing_pos[i] < pst_pos)) {
                    preamble[preamble_idx].bits_pos[bit - 1] =
                        timing_pos[i] % scale;
                    preamble[preamble_idx].mag_sign[bit - 1] = timing_mag[i];
                    preamble[preamble_idx].mag_val[bit - 1] = timing_val[i];
                    if (0 == preamble_pos) {
                        preamble_pos = preamble[preamble_idx].bits_pos[bit - 1];
                    }
                    preamble_bits |= 1;
                    if (pos_last == 0) {
                        pre_pos = timing_pos[i] - delta;
                        pst_pos = timing_pos[i] + delta;
                        pos_last = timing_pos[i];
                    }
                    else {
                        pos_next = ((timing_pos[i] % scale) * WQ_VTB_POS_WERGHT
                            + (pos_last % scale) * (10 - WQ_VTB_POS_WERGHT)) / 10;
                        pos_next += (timing_pos[i] / scale) * scale;
                        pre_pos = pos_next - delta;
                        pst_pos = pos_next + delta;
                        pos_last = timing_pos[i];
                    }
                    break;
                }
            }
            pre_pos -= scale;
            pst_pos -= scale;
        }

        if ((preamble_bits != 0) && (last_preamble_bits != preamble_bits)) {
            preamble[preamble_idx].pos = preamble_pos;
            preamble[preamble_idx].preamble = preamble_bits;
            last_preamble_bits = preamble_bits;
            preamble_idx++;
            //wq_dbg_printf("phase %d, preamble %08x, pos %d.",
            //    phase, preamble_bits, preamble_pos);
        }
        else {
            /* Invalid preamble, print log. */
        }

        if (preamble_idx >= preamble_cnt) {
            break;
        }

        if (cur_pos == end_pos) {
            break;
        }

        cur_pos -= delta * 2;
        cur_pos = max(end_pos, cur_pos);
    }

    wq_dbg_printf("phase %d, found preambles %d.", phase, preamble_idx);

    return preamble_idx;
}

wq_prm_val_t* wq_preamble_make_matchable(wq_prm_val_t* preamble_buf,
    uint32_t buf_len, uint32_t* frm_len, uint32_t* lost_bits,
    uint32_t* pos_bits, uint32_t* neg_bits)
{
    unsigned long* preamble_vector = NULL, cur_preamble = 0;
    wq_prm_val_t* p_prem = NULL, * fin_prm = NULL;
    uint32_t miss_bits_lst = 32, idx, frm_l = 0, miss_bits, toltal_bits = 0;
    float sum_val = 0, avg_val = 0;

    *pos_bits = 0;
    *neg_bits = 0;
    preamble_vector = preuso_noise_sequence_topo();

    for (idx = 0; idx < buf_len; idx++) {
        p_prem = preamble_buf + idx;
        miss_bits = count_num(p_prem->preamble ^ preamble_vector[0]);
        if (miss_bits < miss_bits_lst) {
            frm_l = 1;
            fin_prm = p_prem;
            miss_bits_lst = miss_bits;
            cur_preamble = preamble_vector[0];
        }
        miss_bits = count_num(p_prem->preamble ^ preamble_vector[1]);
        if (miss_bits < miss_bits_lst) {
            frm_l = 3;
            fin_prm = p_prem;
            miss_bits_lst = miss_bits;
            cur_preamble = preamble_vector[1];
        }
    }

    if (NULL == fin_prm) {
        return NULL;
    }

    for (idx = 0; idx < PREAMBLE_LENGTH; idx++) {
        if ((fin_prm->preamble & (1 << idx))
            && (cur_preamble & (1 << idx))) {
            if (fin_prm->mag_sign[idx]) {
                (*pos_bits)++;
            }
            else {
                (*neg_bits)++;
            }
            toltal_bits++;
            sum_val += abs(fin_prm->mag_val[idx]);
        }
    }

    if (toltal_bits > 0) {
        avg_val = sum_val / toltal_bits;
    }

    fin_prm->avg_val = avg_val;

    for (idx = PREAMBLE_LENGTH; idx > 0; idx--) {
        if ((cur_preamble & (1 << (idx - 1)))
            && (fin_prm->preamble & (1 << (idx - 1)))) {
            wq_dbg_printf("preamble %x / %x, change pos %d -> %d",
                fin_prm->preamble, cur_preamble, fin_prm->pos,
                fin_prm->bits_pos[idx - 1]);
            fin_prm->pos = fin_prm->bits_pos[idx - 1];
            break;
        }
    }

    *frm_len = frm_l;
    *lost_bits = miss_bits_lst;

    return fin_prm;
}

/*
There must be a bit '1' in the preamble code. return length of frame,
0 means not found.
*/
uint32_t wq_preamble_detect(rx_topo_init_t* rx, uint32_t phase,
    uint32_t scale, uint32_t delta, wq_prm_val_t* preamble_data)
{
    wq_prm_val_t preamble[PREAMBLE_BIT_GROUP * PREAMBLE_LENGTH] = { 0 };
    wq_prm_val_t* final_preamble = NULL;
    uint32_t preamble_cnt, frame_len, ret = 0;
    uint32_t lost_bits, pos_bits = 0, neg_bits = 0;

    wq_dbg_printf("phase %d scale=%d, delta=%d.", phase, scale, delta);

    preamble_cnt = wq_preamble_scan_rough(rx, phase, scale, delta, preamble,
        sizeof(preamble) / sizeof(preamble[0]));

    final_preamble = wq_preamble_make_matchable(preamble, preamble_cnt,
        &frame_len, &lost_bits, &pos_bits, &neg_bits);

    if (NULL != final_preamble) {
        if (lost_bits <= rx->popcnt_threshold
            && WQ_VTB_AMP_ERR_BITS >= neg_bits) {
            memcpy(preamble_data, final_preamble, sizeof(*final_preamble));
            ret = frame_len;

            wq_info_printf("phase %d preamble found, %08x, "
                "lost bits %d, pos %d, frm_len %d, pos_bits %d, neg_bits %d.",
                phase, final_preamble->preamble, lost_bits, final_preamble->pos,
                frame_len, pos_bits, neg_bits);
        }
        else {
            wq_dbg_printf("phase %d preamble not found, %08x, "
                "lost bits %d, pos %d, frm_len %d, pos_bits %d, neg_bits %d.",
                phase, final_preamble->preamble, lost_bits, final_preamble->pos,
                frame_len, pos_bits, neg_bits);
        }
    }
    else {
        wq_dbg_printf("phase %d preamble not found.", phase);
    }

    return ret;
}

uint32_t decode_preamble_per_phase(rx_topo_init_t* rx, uint8_t phase,
    wq_prm_bit_val_t* p_prm_bit)
{
    uint32_t frame_length, pos = 0;
    wq_prm_val_t fin_preamble;

    rx->receive_bit_num[phase]++;  //number of received bits

    wq_preamble_push_bit(rx, phase, p_prm_bit);

    if (rx->receive_bit_num[phase] < PREAMBLE_LENGTH) {
        return 0; /* Not full preamble data. */
    }

    /* Got 32 bit preamble, detect here. */
    frame_length = wq_preamble_detect(rx, phase, PREAMBLE_SCALE,
        PREAMBLE_SEARCH_DELTA, &fin_preamble);

    /* if find preamble, calculate the message length,
     * and reset related state variables
     */
    if (frame_length > 0) {
        rx->rec_msg_len[phase] = rx->conv->rate * (frame_length + 3);
        rx->packet_det_flag[phase] = 1;
        rx->preamble_try_count[phase] = 0;
        rx->preamble_find_num[phase]++;
        wq_preamble_clear(rx, phase);
        pos = fin_preamble.pos;
        rx->avg_value[phase] = fin_preamble.avg_val;
    }
    else {
        //if not find preamble, return 0
        rx->preamble_try_count[phase]++;
        return 0;
    }

    return pos;
}

uint32_t wq_preamble_copy_2(iot_bits_timming_t* timing,
    wq_prm_bit_val_t* bits)
{
    uint32_t g_idx, b_idx, bits_cnt = 0, detla_scale;
    wq_prm_bit_val_t* p_prm = bits;
    iot_bits_timming_t* p_time_pos = timing;

    for (b_idx = 0; b_idx < PREAMBLE_BIT_LEN; b_idx++) {
        detla_scale = b_idx * PREAMBLE_SCALE;
        for (g_idx = 0; (g_idx < p_prm->bit_cnt)
            && (g_idx < PREAMBLE_BIT_GROUP); g_idx++) {
            p_time_pos->pos = p_prm->bit_pos[g_idx] + detla_scale;
            p_time_pos->sign = p_prm->mag_sign[g_idx];
            p_time_pos->val = p_prm->mag_val[g_idx];
            bits_cnt++;
            if (bits_cnt >= PREAMBLE_BIT_LEN * PREAMBLE_BIT_GROUP) {
                goto copy_out;
            }
            p_time_pos++;
        }
        p_prm++;
    }

copy_out:
    return bits_cnt;
}

uint32_t wq_preamble_scan_rough_2(uint32_t phase,
    iot_bits_timming_t* timming, uint32_t bits_cnt, uint32_t prmb_pttn,
    iot_rx_topo_preamble_t* preamble)
{
    uint32_t pos_cur = 0, pos_end = 0, pos_pre = 0, pos_pst = 0, cur_bit = 0;
    uint32_t preamble_found = 0, bits_error = 0;
    int fin_pos = 0, bit_idx = 0, bit1_cnt = 0, tm_idx = 0;
    float sum_val = 0, temp_ave_val = 0;
    uint32_t bits_cnt_tmp = 0, i = 0;
    iot_rx_topo_preamble_t almost_prmb = { 0 };
    uint16_t bit1_pos_save[PREAMBLE_LENGTH] = { 0 };
    float val_max = 0, val_min = 0;

    if (bits_cnt == 0) {
        return 0;
    }

    /* when signal are separated by two data sets, they need to be merged */
    for (i = bits_cnt - 1; i > 0; i--) {
        val_max = max(abs(timming[i].val), abs(timming[i - 1].val));
        val_min = min(abs(timming[i].val), abs(timming[i - 1].val));
        if (((timming[i].pos % PREAMBLE_SCALE) < WQ_VTB_WAVELET_PEAK_RANGE)
            && ((timming[i - 1].pos % PREAMBLE_SCALE)
        > (PREAMBLE_SCALE - WQ_VTB_WAVELET_PEAK_RANGE))
            && ((timming[i].pos - timming[i - 1].pos)
                < WQ_VTB_WAVELET_PEAK_RANGE)
            && (val_max / val_min < 2)) {
            timming[i].pos = timming[i - 1].pos;
            timming[i].sign = timming[i - 1].sign;
        }
    }

    pos_cur = PREAMBLE_BIT_LEN * PREAMBLE_SCALE - PREAMBLE_SEARCH_DELTA;
    pos_end = PREAMBLE_BIT_LEN * PREAMBLE_SCALE - PREAMBLE_SCALE;

    almost_prmb.err_bits = PREAMBLE_LENGTH;

    while (pos_cur >= pos_end) {
        preamble_found = 0;
        fin_pos = -1;
        bits_error = 0;
        bit1_cnt = 0;
        sum_val = 0;
        pos_pre = pos_cur - PREAMBLE_SEARCH_DELTA;
        pos_pst = pos_cur + PREAMBLE_SEARCH_DELTA;
        bits_cnt_tmp = bits_cnt;
        temp_ave_val = 0.0;
        memset(bit1_pos_save, 0xff, sizeof(bit1_pos_save));

        for (bit_idx = PREAMBLE_LENGTH - 1; bit_idx >= 0; bit_idx--) {
            /* Current bit */
            cur_bit = 0;

            /* Search if there is a bit in range. */
            for (tm_idx = bits_cnt_tmp - 1; tm_idx >= 0; tm_idx--) {
                if (timming[tm_idx].pos < pos_pre) {
                    /* No bit in range anymore. */
                    break;
                }
                if (timming[tm_idx].pos <= pos_pst) {
                    /* Got one bit in range. */
                    cur_bit = 1;
                    preamble_found |= (cur_bit << bit_idx);
                    bits_cnt_tmp = tm_idx;
                    bit1_pos_save[bit_idx] = tm_idx;
                    break;
                }
            }

            if (cur_bit == ((prmb_pttn >> bit_idx) & 0x1)) {
                if (cur_bit) {
                    /* Got a correct bit-1 */
                    if (fin_pos < 0) {
                        fin_pos = timming[tm_idx].pos % PREAMBLE_SCALE;
                    }
                    /* update pos */
/* whether alpha is enabled to smooth pos parameters while looking for preamble */
#if 1
                    if (timming[tm_idx].pos > PREAMBLE_SEARCH_DELTA) {
                        pos_pre = timming[tm_idx].pos - PREAMBLE_SEARCH_DELTA;
                    }
                    else {
                        pos_pre = 0;
                    }
                    pos_pst = timming[tm_idx].pos + PREAMBLE_SEARCH_DELTA;
#else
                    uint32_t pos_next = 0, pos_last = 0;
                    if (pos_last == 0) {
                        if (timming[tm_idx].pos > PREAMBLE_SEARCH_DELTA) {
                            pos_pre = timming[tm_idx].pos - PREAMBLE_SEARCH_DELTA;
                        }
                        else {
                            pos_pre = 0;
                        }
                        pos_pst = timming[tm_idx].pos + PREAMBLE_SEARCH_DELTA;
                        pos_last = timming[tm_idx].pos;
                    }
                    else {
                        pos_next = ((timming[tm_idx].pos % PREAMBLE_SCALE)
                            * WQ_VTB_POS_WERGHT + (pos_last % PREAMBLE_SCALE)
                            * (10 - WQ_VTB_POS_WERGHT)) / 10;
                        pos_next += (timming[tm_idx].pos / PREAMBLE_SCALE)
                            * PREAMBLE_SCALE;
                        if (pos_next > PREAMBLE_SEARCH_DELTA) {
                            pos_pre = pos_next - PREAMBLE_SEARCH_DELTA;
                        }
                        else {
                            pos_pre = 0;
                        }
                        pos_pst = pos_next + PREAMBLE_SEARCH_DELTA;
                        pos_last = timming[tm_idx].pos;
                    }
#endif
                    /* ave of value */
                    sum_val += abs(timming[tm_idx].val);
                    bit1_cnt++;
                }
                else {
                    /* TODO : Got a correct bit-0 */
                }
            }
            else {
                /* Got a error bit. */
                bits_error++;
            }

            /* Search next bits */
            if (pos_pre > PREAMBLE_SCALE) {
                pos_pre -= PREAMBLE_SCALE;
            }
            else {
                pos_pre = 0;
            }
            if (pos_pst > PREAMBLE_SCALE) {
                pos_pst -= PREAMBLE_SCALE;
            }
            else {
                pos_pst = 0;
            }
        }

        /* Valid almost preabmle */
        if (bit1_cnt > 0) {
            temp_ave_val = sum_val / bit1_cnt;
            for (i = 0; i < PREAMBLE_LENGTH; i++) {
                if ((preamble_found >> i) & 0x1) {
                    if (abs(timming[bit1_pos_save[i]].val * 3)
                        <= temp_ave_val) {
                        preamble_found &= ~(1 << i);
                    }
                }
            }
            bits_error = count_num(preamble_found ^ prmb_pttn);
            if (almost_prmb.err_bits > bits_error) {
                /* Choose one with less error bits. */
                almost_prmb.pos = (uint32_t)fin_pos;
                almost_prmb.val = temp_ave_val;
                almost_prmb.err_bits = bits_error;
                almost_prmb.prmb = preamble_found;
            }
        }

        /* Search next preamble. */
        if (pos_cur == pos_end) {
            break;
        }
        pos_cur -= PREAMBLE_SEARCH_DELTA * 2;
        pos_cur = max(pos_end, pos_cur);
    }

    memcpy(preamble, &almost_prmb, sizeof(almost_prmb));

    wq_dbg_printf("phase:%d, almost preamble %08x, lost bits %d, pos %d, val %d",
        phase, preamble->prmb, preamble->err_bits, preamble->pos,
        (int)preamble->val);

    return ((almost_prmb.err_bits < PREAMBLE_LENGTH) ? 1 : 0);
}

uint32_t wq_preamble_detect_2(rx_topo_init_t* rx, uint32_t phase,
    iot_rx_topo_preamble_t* preamble)
{
    uint16_t bits_cnt = 0, data_len = 0;
    iot_bits_timming_t timming[PREAMBLE_BIT_LEN * PREAMBLE_BIT_GROUP] = { 0 };
    unsigned long* preamble_vector = preuso_noise_sequence_topo();
    iot_rx_topo_preamble_t tmp_prmb[2] = { 0 }; /* 2 preamble_vector */

    bits_cnt = wq_preamble_copy_2(timming, rx->preamble_bits[phase]);

    (void)wq_preamble_scan_rough_2(phase, timming, bits_cnt,
        (uint32_t)preamble_vector[1], &tmp_prmb[1]);

    (void)wq_preamble_scan_rough_2(phase, timming, bits_cnt,
        (uint32_t)preamble_vector[0], &tmp_prmb[0]);

    if ((tmp_prmb[0].err_bits < tmp_prmb[1].err_bits)
        && (tmp_prmb[0].err_bits <= rx->popcnt_threshold)) {
        data_len = 1;
        memcpy(preamble, &tmp_prmb[0], sizeof(*preamble));
    }
    else if (tmp_prmb[1].err_bits <= rx->popcnt_threshold) {
        data_len = 3;
        memcpy(preamble, &tmp_prmb[1], sizeof(*preamble));
    }
    else {
        data_len = 0;
    }

    if (data_len > 0) {
            wq_info_printf("phase %d preamble found, %08x, lost bits %d, pos %d, "
            "ave_val:%d, frm_len %d.", phase, preamble->prmb,
            preamble->err_bits, preamble->pos, (int)preamble->val, data_len);
    }
    else {
        wq_dbg_printf("phase %d preamble not found, %08x, lost bits %d, "
            "pos %d, ave_val:%d, frm_len %d.", phase, preamble->prmb,
            preamble->err_bits, preamble->pos, (int)preamble->val, data_len);
    }

    return data_len;
}

uint32_t decode_preamble_per_phase_2(rx_topo_init_t* rx, uint8_t phase,
    wq_prm_bit_val_t* p_prm_bit)
{
    uint32_t frame_length, pos = 0;
    iot_rx_topo_preamble_t fin_preamble;

    rx->receive_bit_num[phase]++;  //number of received bits

    wq_preamble_push_bit(rx, phase, p_prm_bit);

    if (rx->receive_bit_num[phase] < PREAMBLE_LENGTH) {
        return 0; /* Not full preamble data. */
    }

    /* Got 32 bit preamble, detect here. */
    frame_length = wq_preamble_detect_2(rx, phase, &fin_preamble);

    //find preamble, calculate the message length, reset related state variables
    if (frame_length > 0) {
        rx->rec_msg_len[phase] = rx->conv->rate * (frame_length + 3);
        rx->packet_det_flag[phase] = 1;
        rx->preamble_try_count[phase] = 0;
        rx->preamble_find_num[phase]++;
        wq_preamble_clear(rx, phase);
        pos = fin_preamble.pos;
        rx->avg_value[phase] = fin_preamble.val;
    }
    else {
        //if not find preamble, return 0
        rx->preamble_try_count[phase]++;
        return 0;
    }

    return pos;
}

uint8_t decode_data_per_phase(rx_topo_init_t* rx, uint8_t phase,
    uint8_t mask, uint8_t* decoded_data, uint8_t* decoded_data_len,
    uint8_t* reset)
{
    uint8_t flag = 0;

    wq_dbg_printf("phase %d, mask=%d.", phase, mask);

    rx->msg_count[phase]++;
    if (rx->packet_det_flag[phase] && (rx->msg_count[phase] <= (rx->rec_msg_len[phase] * 8))) {
        uint8_t byte_index = (rx->msg_count[phase] - 1) / 8,
            bit_index_in_byte = (rx->msg_count[phase] - 1) % 8;
        rx->phase_data_buf[phase][byte_index] += mask << bit_index_in_byte;
        return flag;
    }

    wq_dbg_printf("phase %d frame complate, msg_count=%d.",
        phase, rx->msg_count[phase]);

    deinterleave(rx->phase_data_buf[phase], rx->rec_msg_len[phase],
        decoded_data);

    uint8_t decode_msg_len_with_crc = rx->rec_msg_len[phase] / rx->conv->rate - 1;
    uint8_t mask1 = 0x80;

    for (uint16_t j = 0; j < rx->rec_msg_len[phase] * 8; j++) {
        rx->phase_data_buf[phase][j] = (decoded_data[j / 8] & mask1) ? 255 : 0;
        mask1 >>= 1;
        if (mask1 == 0)
            mask1 = 0x80;
    }

    correct_convolutional_decode_soft(rx->conv, rx->phase_data_buf[phase],
        rx->rec_msg_len[phase] * 8, decoded_data);

    descramble(descramble_polynomial, decoded_data, decode_msg_len_with_crc,
        rx->phase_data_buf[phase], rx->conv);

    flag = crc_result_check(crc16_polynomial, rx->phase_data_buf[phase],
        decode_msg_len_with_crc, decoded_data, rx->conv);
    decoded_data_len[0] = decode_msg_len_with_crc - 2;

    if (flag) {
        rx->crc_success_num[phase]++;
        wq_info_printf("phase %d frame found!%x, %x, %x.",
            phase, decoded_data[0], decoded_data[1], decoded_data[2]);
    }
    else {
        wq_info_printf("phase %d frame broken!", phase);
    }
    *reset = 1;
    return flag;
}

void wq_vtb_adc_data_format_print(int32_t* buf, uint32_t len,
    uint8_t phase)
{
    static uint32_t pack_cnt = 1;

    iot_wq_printf("<-- %s --> idx %d, len %lu, phase %lu\n",
        __FUNCTION__, pack_cnt++, len, phase);

    for (uint32_t i = 0; i < len; ++i) {
        iot_wq_printf("%d,", buf[i]);
        if (((i + 1) % 64) == 0 && i != 0) {
            iot_wq_printf("...\n");
        }
    }

    iot_wq_printf("...\n");

    return;
}

uint8_t wq_vtb_topo_bit_rec(uint8_t* rx_ptr, uint8_t phase,
    int32_t* sample_data, uint16_t data_len, uint8_t sample_order,
    uint8_t* decoded_data, uint8_t* decoded_data_len, int8_t* offset,
    int16_t* data_offset, uint32_t counter)
{
    (void)counter;
    uint8_t decode_flag = 0;//packet decode flag
    uint8_t reset = 0;//reset flag
    int32_t pos = 0;
    uint8_t i = 0;
    rx_topo_init_t* rx = (rx_topo_init_t*)rx_ptr;
    uint16_t pos_smooth_temp = 0;
#if(TOPO_VERSION >= TOPO_V3_4)
    uint16_t diff_pos_value = 0;
    uint16_t diff_pos_value_last = 0;
    uint8_t almost_pos_index = 0xff;
#endif
    /*
        if (phase == 0) {
            wq_vtb_adc_data_format_print(sample_data, data_len, phase);
        }
    */
    wq_prm_bit_val_t preamble_bit; /* New comes preamble bit. */

    wq_dbg_printf("phase=%d, sample_data=%p, data_len=%d, sample_order=%d.",
        phase, sample_data, data_len, sample_order);

    rx->call_time[phase]++; /* Not used. */

    if (!rx->packet_det_flag[phase]) {
        pos = -1;
    }
    else {
        pos = rx->correct_offset[phase];
    }

    /* demodulate sample data based on wavelet analysis.
     * the results are stored in mask.
     */
    uint8_t mask = disturb_detection(phase, sample_data, data_len,
        sample_order, pos, rx->avg_value[phase], &preamble_bit);

    wq_dbg_printf("phase %d bit, cout %d, pos:%d,%d,%d, sign:%d,%d,%d,"
        "val:%d,%d,%d", phase, preamble_bit.bit_cnt,
        preamble_bit.bit_pos[0], preamble_bit.bit_pos[1],
        preamble_bit.bit_pos[2], preamble_bit.mag_sign[0],
        preamble_bit.mag_sign[1], preamble_bit.mag_sign[2],
        (int)preamble_bit.mag_val[0], (int)preamble_bit.mag_val[1],
        (int)preamble_bit.mag_val[2]);

    *offset = 0;

    if (!rx->packet_det_flag[phase]) {
        pos = decode_preamble_per_phase_2(rx, phase, &preamble_bit);
        if (rx->packet_det_flag[phase]) {
            rx->correct_offset[phase] = pos;
            *offset = (PREAMBLE_SCALE / 2 - rx->correct_offset[phase]) / 128;
            rx->correct_offset[phase] = PREAMBLE_SCALE / 2
                - ((PREAMBLE_SCALE / 2 - rx->correct_offset[phase]) % 128);
        }
    }
    else {
        if (mask) {
            for (i = 0; i < preamble_bit.bit_cnt; i++) {
                if (rx->first_mask[phase]) {
                    pos_smooth_temp = rx->correct_offset[phase];
                    rx->first_mask[phase] = 0;
                    rx->pos_smooth[phase] = (uint16_t)rx->correct_offset[phase];
                }
                else {
                    pos_smooth_temp = (rx->pos_smooth[phase] * 7
                        + preamble_bit.bit_pos[i] * 3) / 10;
                }
#if(TOPO_VERSION >= TOPO_V3_4)
                diff_pos_value =
                    abs(pos_smooth_temp - preamble_bit.bit_pos[i]);
                if (diff_pos_value < 128) {
                    if (diff_pos_value_last == 0
                        || diff_pos_value_last > diff_pos_value) {
                        diff_pos_value_last = diff_pos_value;
                        rx->pos_smooth[phase] = pos_smooth_temp;
                        rx->correct_offset[phase] = preamble_bit.bit_pos[i];
                        almost_pos_index = i;
                    }
                }
            }
            if (almost_pos_index != 0xff) {
                rx->sign_sum[phase] += preamble_bit.mag_sign[almost_pos_index];
                rx->bit1_counter[phase]++;
            }
#else if(TOPO_VERSION >= TOPO_V3_3)
                if (preamble_bit.bit_pos[i] < (pos_smooth_temp + 128)
                    && preamble_bit.bit_pos[i] > (pos_smooth_temp - 128)) {
                    rx->pos_smooth[phase] = pos_smooth_temp;
                    rx->correct_offset[phase] = preamble_bit.bit_pos[i];
                    if (preamble_bit.mag_sign[i] == 1) {
                        rx->act_bits[phase]++;
                    }
                    else {
                        rx->neg_bits[phase]++;
                    }
                    break;
                }
            }
#endif
        }
        wq_dbg_printf("phase %d bit, mask %d, cout %d, pos:%d,%d,%d, smooth pos:%d, next %d.",
            phase, mask, preamble_bit.bit_cnt, preamble_bit.bit_pos[0],
            preamble_bit.bit_pos[1], preamble_bit.bit_pos[2],
            rx->pos_smooth[phase], rx->correct_offset[phase]);

        //try to decode the current data in the buffer
        decode_flag = decode_data_per_phase(rx, phase, mask, decoded_data,
            decoded_data_len, &reset);
        if (decode_flag) {
#if(TOPO_VERSION >= TOPO_V3_4)
            wq_dbg_printf("phase %d sign sum:%d, bit1 counter:%d, "
                "probability of upward:%d%%", phase,
                rx->sign_sum[phase], rx->bit1_counter[phase],
                (int)(rx->sign_sum[phase] / rx->bit1_counter[phase]));
            if (rx->sign_sum[phase] * 3 < rx->bit1_counter[phase] * 100 * 2) {
                wq_dbg_printf("data will be discarded");
                decode_flag = 0;
            }
#else if(TOPO_VERSION >= TOPO_V3_3)
            wq_dbg_printf("phase %d act_bits:%d, neg_bits:%d \n", phase,
                rx->act_bits[phase], rx->neg_bits[phase]);
            if ((rx->neg_bits[phase] * 3)
                >= (rx->act_bits[phase] + rx->neg_bits[phase])) {
                decode_flag = 0;
                wq_dbg_printf("data will be discarded");
            }
#endif
        }
    }
#if 0 /* TODO : Remove delta for this point. Coming soon */
    if ((NULL != g_wq_vtb_topo_delta) && (0 != g_wq_vtb_topo_delta->average)) {
        *data_offset = g_wq_vtb_topo_delta->average * 25 / 78 - 6400;
        wq_dbg_printf("average = %d.", g_wq_vtb_topo_delta->average);
    }
    else {
        *data_offset = 10;
    }
#else
    * data_offset = 10;
#endif

    //if reset flag is ture, reset related state variables
    if (reset) {
        wq_dbg_printf("reset flags.");
        for (uint16_t i = 0; i < 160; i++) {
            rx->phase_data_buf[phase][i] = 0;
        }
#if(TOPO_VERSION >= TOPO_V3_4)
        rx->bit1_counter[phase] = 0;
        rx->sign_sum[phase] = 0;
#else if(TOPO_VERSION >= TOPO_V3_3)
        rx->act_bits[phase] = 0;
        rx->neg_bits[phase] = 0;
#endif
        rx->first_mask[phase] = 1;
        rx->pos_smooth[phase] = 0;
        rx->call_time[phase] = 0;
        rx->msg_count[phase] = 0;
        rx->rec_msg_len[phase] = 0;
        rx->packet_det_flag[phase] = 0;
        rx->preamble_try_count[phase] = 0;
        rx->receive_bit_num[phase] = 0;
        correct_convolutional_destroy(rx->conv);
        rx->conv = correct_convolutional_create(2, 7,
            correct_conv_r12_7_polynomial);
    }

    wq_dbg_printf("phase %d decode_flag=%d, *decoded_data_len=%d, *offset=%d, "
        "*data_offset=%d.", phase, decode_flag, *decoded_data_len,
        *offset, *data_offset);

    //return decode result
    return decode_flag;
}

void wq_vtb_topo_rx_destroy(uint8_t* rx_ptr)
{
    rx_topo_init_t* rx = (rx_topo_init_t*)rx_ptr;
    if (rx->conv != NULL) {
        correct_convolutional_destroy(rx->conv);
    }
    if (rx != NULL) {
        free(rx);
    }
}

#endif /* PLC_SUPPORT_HW_TOPO */
