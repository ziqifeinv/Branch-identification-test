#ifndef CORRECT_CONVOLUTIONAL_H
#define CORRECT_CONVOLUTIONAL_H
#include "conv_common.h"
#include "bit.h"
#include "metric.h"
#include "lookup.h"
#include "history_buffer.h"
#include "error_buffer.h"

struct correct_convolutional {
    const uint16_t *table;  // size 2**order
    size_t rate;                // e.g. 2, 3...
    size_t order;               // e.g. 7, 9...
    uint16_t numstates;     // 2**order
    bit_writer_t *bit_writer;
    bit_reader_t *bit_reader;

    bool has_init_decode;
    distance_t *distances;
    pair_lookup_t pair_lookup;
    soft_measurement_t soft_measurement;
    history_buffer *history_buffer;
    error_buffer_t *errors;
};

correct_convolutional *_correct_convolutional_init(correct_convolutional *conv,
                                                   size_t rate, size_t order,
                                                   const polynomial_t *poly);

void correct_convolutional_destroy(correct_convolutional *conv);

#endif

