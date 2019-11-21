#ifndef CORRECT_CONVOLUTIONAL_LOOKUP
#define CORRECT_CONVOLUTIONAL_LOOKUP
#include "conv_common.h"


typedef uint16_t distance_pair_key_t;
typedef uint32_t output_pair_t;
typedef uint32_t distance_pair_t;

typedef struct {
    distance_pair_key_t *keys;
    output_pair_t *outputs;
    output_pair_t output_mask;
    uint16_t output_width;
    size_t outputs_len;
    distance_pair_t *distances;
} pair_lookup_t;

void fill_table(uint16_t order,
                uint16_t rate,
                const polynomial_t *poly,
                uint16_t *table);

pair_lookup_t pair_lookup_create(uint16_t rate,
                                 uint16_t order,
                                 const uint16_t *table);
void pair_lookup_destroy(pair_lookup_t pairs);
void pair_lookup_fill_distance(pair_lookup_t pairs, distance_t *distances);
#endif
