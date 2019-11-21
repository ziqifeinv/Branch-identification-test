#include "scramble.h"
#include "bit.h"

void scramble(   const scramble_polynomial_t poly,
                    uint8_t *msg,
            uint8_t msg_len,
            uint8_t *scrambled_msg,
            correct_convolutional *conv){
    scramble_polynomial_t register_state = 0; 

    //bit_writer_t *bit_writer = bit_writer_create(NULL, 0);
    //bit_reader_t *bit_reader = bit_reader_create(NULL, 0);

    bit_writer_reconfigure(conv->bit_writer, scrambled_msg, msg_len);

    bit_reader_reconfigure(conv->bit_reader, msg, msg_len);    
    
    uint16_t temp; uint8_t mask;
    for(uint16_t i = 0; i < 8 * msg_len; i++){
        temp = register_state & poly ;        
        mask = (temp >> 3 & 1) ^ (temp >> 6 & 1);        
        register_state = register_state << 1 & 0x7f;        
        mask ^= bit_reader_read(conv->bit_reader, 1);
        //printf("%d\t",mask);
        bit_writer_write(conv->bit_writer, mask, 1);
        register_state += mask;        
    }
}
