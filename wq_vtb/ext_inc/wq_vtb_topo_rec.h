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

#ifndef WQ_VTB_TOPO_REC_H
#define WQ_VTB_TOPO_REC_H

#ifdef __cplusplus
extern "C" {
#endif

#define WQ_VTB_AC_PARAM_DIV   (25) /* 50 +- 2 HZ , 2 / 50 = 1 / 25*/

#define WQ_VTB_AMP_ERR_BITS     2

/* define max number of different bits when searching preamble */
#define WQ_VTB_TOPO_RX_NUM_MAX      4

/* define phase lines */
#define WQ_VTB_TOPO_RX_PHASE_A      0
#define WQ_VTB_TOPO_RX_PHASE_B      1
#define WQ_VTB_TOPO_RX_PHASE_C      2

/**
 * @brief: receive related pointers and variables initializaiton.
 * @param demod_th: demodulation threshold.
 * @param pop_num: max number of different bits when searching preamble.
 * @return value: the pointer to the rx struct.
 */
uint8_t *wq_vtb_topo_rx_create(uint8_t pop_num);

/**
 * @brief: wq_vtb_topo_zc_delta_handle()-deal with zc delta data.
 * @param data: pointer to an array.
 * @param data_len: data length.
 * @param last_counter:last delta data timer counter
 * @return 0.
 */
uint8_t wq_vtb_topo_zc_delta_handle(uint8_t *data, uint8_t data_len,
    uint32_t last_counter);

/**
 * @brief received data decode function
 * @param rx: receiver related struct.
 * @param phase: input data phase (A,B,C) indicator, 0 ~ 2.
 * @param sample_data: input sample data, ADC.
 * @param data_len: input data length. About 6400 = 25 * 256, 25 cycle.
 * @param sample_order: sample data order for each period. Num of data point,
    power() of 2.
 * @param decoded_data: the pointer to the decoded data, max length 20 bytes.
    Recovered data.
 * @param decoded_data_len: the pointer to the decoded data length. Length of
    recover data.
 * @param offset: the pointer of the period offset.
    Num of cycles to adjust the disturbance position.
    position move to the center.
 * @param data_offset: the pointer of the data offset. Compensate frequency
    offset of AC&ADC.

 * @return value: decode success flag.
 */
uint8_t wq_vtb_topo_bit_rec(uint8_t* rx_ptr, uint8_t phase,
    int32_t* sample_data, uint16_t data_len, uint8_t sample_order,
    uint8_t* decoded_data, uint8_t* decoded_data_len, int8_t* offset,
    int16_t* data_offset, uint32_t counter);

/**
 * @brief: receive related pointers and variables destroy.
 * @param rx: the pointer to the rx struct.
 */
void wq_vtb_topo_rx_destroy(uint8_t *rx_ptr);

#ifdef __cplusplus
}
#endif

#endif /* WQ_VTB_TOPO_REC_H */
