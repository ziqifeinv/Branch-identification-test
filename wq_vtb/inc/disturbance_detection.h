#ifndef DISTURBANCE_DETECTION_H
#define DISTURBANCE_DETECTION_H

#include <stdint.h>
#include "preamble_generator.h"

#define PREAMBLE_BIT_GROUP      3
#define PREAMBLE_PHASE_NUM      3
#define PREAMBLE_BIT_LEN        (PREAMBLE_LENGTH + 1) /* 32 + 1 temp. */

#define PREAMBLE_SCALE          (3200 + 5)
#define PREAMBLE_SEARCH_DELTA   64

typedef struct _wq_preamble_bit_value_t {
    uint16_t bit_pos[PREAMBLE_BIT_GROUP];
    uint8_t mag_sign[PREAMBLE_BIT_GROUP]; /* 0: negative, else : positive */
    float mag_val[PREAMBLE_BIT_GROUP];
    uint8_t bit_cnt;
} wq_prm_bit_val_t;

typedef struct _wq_preamble_value_t {
    uint32_t preamble;  /* preamble found */
    uint32_t pos;       /* finally pos. */
    float avg_val;      /* finally value. */
    uint8_t mag_sign[PREAMBLE_LENGTH];  /* 0: negative, else : positive */
    uint16_t bits_pos[PREAMBLE_LENGTH]; /* pos of each bit */
    float mag_val[PREAMBLE_LENGTH];
}wq_prm_val_t;

uint8_t disturb_detection(uint32_t phase, int32_t *sample_data, uint16_t data_len,
    uint8_t sample_order, int32_t exp_pos, float ave_mag, wq_prm_bit_val_t *preamble_bits_data);


#endif

