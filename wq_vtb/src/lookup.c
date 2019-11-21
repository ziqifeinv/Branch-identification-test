#include "convolutional.h"
#include "lookup.h"

// table has numstates rows
// each row contains all of the polynomial output bits concatenated together
// e.g. for rate 2, we have 2 bits in each row
// the first poly gets the LEAST significant bit, last poly gets most significant
void fill_table(uint16_t rate,
                uint16_t order,
                const polynomial_t *poly,
                uint16_t *table) {
    for (shift_register_t i = 0; i < 1 << order; i++) {
        uint16_t out = 0;
        uint16_t mask = 1;
        for (size_t j = 0; j < rate; j++) {
            out |= (count_num(i & poly[j]) % 2) ? mask : 0;
            mask <<= 1;
        }
        table[i] = out;
    }
}




pair_lookup_t pair_lookup_create(uint16_t rate,
                                 uint16_t order,
                                 const uint16_t *table) {
    pair_lookup_t pairs;

    pairs.keys = malloc(sizeof(uint16_t) * (1 << (order - 1)));
    pairs.outputs = malloc((1 << (rate * 2)) * sizeof(uint16_t));
    //uint16_t *inv_outputs = malloc((1 << (rate * 2)) * sizeof(uint16_t));
    uint16_t temp = (1 << (rate * 2));
    uint16_t inv_outputs[16] = { 0 };
    memset(pairs.keys, 0, sizeof(pairs.keys));
    memset(pairs.outputs, 0, sizeof(pairs.outputs));
    //memset(inv_outputs, 0, sizeof(inv_outputs));
    uint16_t output_counter = 1;
    // for every (even-numbered) shift register state, find the concatenated output of the state
    //   and the subsequent state that follows it (low bit set). then, check to see if this
    //   concatenated output has a unique key assigned to it already. if not, give it a key.
    //   if it does, retrieve the key. assign this key to the shift register state.
    for (uint16_t i = 0; i < (1 << (order - 1)); i++) {
        // first get the concatenated pair of outputs
        uint16_t out = table[i * 2 + 1];
        out <<= rate;
        out |= table[i * 2];

        // does this concatenated output exist in the outputs table yet?
        if (!inv_outputs[out]) {
            // doesn't exist, allocate a new key
            inv_outputs[out] = output_counter;
            pairs.outputs[output_counter] = out;
            output_counter++;
        }
        // set the opaque key for the ith shift register state to the concatenated output entry
        pairs.keys[i] = inv_outputs[out];
    }
    pairs.outputs_len = output_counter;
    pairs.output_mask = (1 << (rate)) - 1;
    pairs.output_width = rate;
    pairs.distances = malloc( pairs.outputs_len * sizeof(distance_pair_t));
    memset(pairs.distances, 0, sizeof(pairs.distances));
    //free(inv_outputs);
    return pairs;
}

void pair_lookup_destroy(pair_lookup_t pairs) {
    free(pairs.keys);
    free(pairs.outputs);
    free(pairs.distances);
}

void pair_lookup_fill_distance(pair_lookup_t pairs, distance_t *distances) {
    for (uint16_t i = 1; i < pairs.outputs_len; i += 1) {
        output_pair_t concat_out = pairs.outputs[i];
        uint16_t i_0 = concat_out & pairs.output_mask;
        concat_out >>= pairs.output_width;
        uint16_t i_1 = concat_out;

        pairs.distances[i] = (distances[i_1] << 16) | distances[i_0];
    }
}