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

#ifndef WQ_VTB_DATA_TO_BIT_H
#define WQ_VTB_DATA_TO_BIT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief: transmit related pointers and variables initializaiton.
 * @return otherwise: handle of vtb transmit.
 * @return NULL: create fail.
 */
uint8_t *wq_vtb_tx_create();

/**
 * @brief: transmit data encode function by using BCC, maximum data size is
 *         7 bytes.
 * @param t_handle:  handle of vtb transmit.
 * @param buf:  the pointer to the cache buffer that stores (to be
 *              encoded and encoded) data, the max length of cache
 *              buffer is exactly 24 bytes.
 * @param data_len:  to be encoded data length, no greater than 7 bytes.
 * @return value: encoded data length, default with 24 bytes.
 */
uint8_t wq_vtb_data_encode(uint8_t *t_handle, uint8_t *buf,
    uint8_t data_len);

/**
 * @brief encode data length calculation, this function will not be
 *        called with current configuration.
 * @param data_len:  to be encoded data length, no greater than 7 bytes.
 * @return value: encoded data length.
 */
uint8_t wq_vtb_get_encode_data_len(uint8_t data_len);

/**
 * @brief: transmit related pointers and variables destroy.
 * @param tx: the pointer to the tx struct.
 */
void wq_vtb_tx_destroy(uint8_t *t_handle);

#ifdef __cplusplus
}
#endif

#endif /* WQ_VTB_DATA_TO_BIT_H */
