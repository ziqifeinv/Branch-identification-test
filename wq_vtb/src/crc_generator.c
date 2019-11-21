#include "crc_generator.h"
#include "bit.h"
//#include "conv_common.h"

// this function is used to calculate the crc result of transmit message,
// and then append the crc result at the end of message
void crc_calculation(      const crc_polynomial_t poly,                      
                             uint8_t *msg,
                    uint16_t msg_len,
                        uint8_t *msg_crc,
                correct_convolutional *conv){
    uint8_t register_len = 16;
    // initialize shift register with 16 bits '1'
    crc_shift_register_t crc_shift_register = 0xffff;    

    //bit_writer_t *bit_writer = bit_writer_create(NULL, 0);
    //bit_reader_t *bit_reader = bit_reader_create(NULL, 0);

    bit_writer_reconfigure(conv->bit_writer, msg_crc, msg_len + register_len/8);

    bit_reader_reconfigure(conv->bit_reader, msg, msg_len);  
    uint8_t left_bit = 0;
    for(uint16_t i = 0; i < 8 * msg_len; i++){
        //printf("resister state currently is %4x\n",crc_shift_register);
        left_bit = crc_shift_register >> (register_len-1) & 1;     
        crc_shift_register = crc_shift_register << 1 ^ 1 << (register_len);
        //printf(":%d\t",bit_reader_read(conv->bit_reader, 1)); 

        // please note that read MSB first for each byte       
        if (bit_reader_read(conv->bit_reader, 1) ^ left_bit){
            crc_shift_register =  crc_shift_register ^ poly ^ 1 << (register_len);
        }
        //printf("resister state currently is %4x\n",crc_shift_register);
        //bit_writer_write(conv->bit_writer, msg[i], 1);                     
    }
    uint8_t crc_rigister_byte[2];
    crc_rigister_byte[0] = (crc_shift_register >> 8) & 0xff;
    crc_rigister_byte[1] = crc_shift_register & 0xff;
    
    for(uint16_t i = 0; i < msg_len + register_len/8; i++){
        msg_crc[i] = i < msg_len ? msg[i] : crc_rigister_byte[i-msg_len];
    }
   
}

