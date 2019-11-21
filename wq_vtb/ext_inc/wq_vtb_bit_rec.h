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

#ifndef WQ_VTB_BIT_REC_H
#define WQ_VTB_BIT_REC_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief: receive related pointers and variables initializaiton.
 * @return otherwise: handle of vtb receive.
 * @return NULL: creat fail.
 */
uint8_t* wq_vtb_rx_create();

/**
 * @brief received data decode function
 * @param r_handle:  handle of vtb receive.
 * @param delta:  receive data of time interval with us unit.
 * @param decoded_data:  the pointer to the decoded data, max length 30 bytes.
 * @param decoded_data_len:  the pointer to the decoded data length.
 * @return value:   decode success flag.
 */
uint8_t wq_vtb_bit_rec(uint8_t *r_handle, uint16_t delta,
    uint8_t *decoded_data, uint8_t *decoded_data_len);

/**
 * @brief: receive related pointers and variables destroy.
 * @param r_handle:  handle of vtb receive.
 */
void wq_vtb_rx_destroy(uint8_t *r_handle);

#ifdef __cplusplus
}
#endif

#endif /* WQ_VTB_BIT_REC_H */
