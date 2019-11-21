#ifndef SYNCRONIZATION_H
#define SYNCRONIZATION_H
#include <stdint.h>


#define MAX_DETECTION_LEN 10000

uint32_t get_potential_preamble(uint8_t *demod_msg         );

uint32_t get_potential_preamble_topo(uint8_t *demod_msg         );


void update_preamble_register(uint8_t *dem_data,
                                        uint8_t current_data);

void update_preamble_register_right(uint8_t *dem_data,
                                        uint8_t current_data);

void update_preamble_register_right_topo(uint8_t *dem_data,
                                        uint8_t current_data);


#endif
