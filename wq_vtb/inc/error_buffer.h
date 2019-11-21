#ifndef ERROR_BUFFER_H
#define ERROR_BUFFER_H


typedef struct {
    uint16_t index;
    distance_t *errors[2];
    uint16_t num_states;

    const distance_t *read_errors;
    distance_t *write_errors;
} error_buffer_t;

error_buffer_t *error_buffer_create(uint16_t num_states);
void error_buffer_destroy(error_buffer_t *buf);
void error_buffer_reset(error_buffer_t *buf);
void error_buffer_swap(error_buffer_t *buf);
#endif