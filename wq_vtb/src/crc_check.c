#include "crc_check.h"
#include "crc_generator.h"
#include "bit.h"

uint8_t crc_result_check(      const crc_polynomial_t poly,                      
                             uint8_t *msg_crc,
                    uint16_t msg_len,
                        uint8_t *msg,
                correct_convolutional *conv){    
    uint8_t register_len = 16;
    uint8_t receive_flag =0;
    // initialize shift register with 16 bits '1'
    crc_shift_register_t crc_shift_register = 0xffff;    

    //bit_writer_t *bit_writer = bit_writer_create(NULL, 0);
    //bit_reader_t *bit_reader = bit_reader_create(NULL, 0);

    bit_writer_reconfigure(conv->bit_writer, msg, msg_len - register_len/8);

    bit_reader_reconfigure(conv->bit_reader, msg_crc, msg_len);  
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

    //printf("register state is:%4x\n",crc_shift_register);
    
    if (!crc_shift_register){
        receive_flag = 1;
        for(uint16_t i = 0; i < msg_len - register_len/8; i++){
            msg[i] = msg_crc[i];
        }
    }
    else{
        msg = NULL;
    }
    //printf("%d\n",receive_flag[0]);
    return receive_flag;       

}
