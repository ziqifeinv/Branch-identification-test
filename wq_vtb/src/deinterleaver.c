#include "deinterleaver.h"

void deinterleave(uint8_t *msg,
                      uint16_t msg_len,
                        uint8_t *deinterleaved_msg){
    deinterleaver_matrix_t interleaver_matrix[][4] = {{1,2,3,4},{5,6,7,8},{9,10,11,12},{13,14,15,16}};

    uint8_t position, current_bit;

    for(uint16_t i = 0; i < msg_len >> 1; i++){
        unsigned int short read_dual_bytes = 0, deinterlev_dual_bytes = 0;        
        read_dual_bytes = (short)( msg[2*i+1] << 8) | msg[2*i];        
        for(uint16_t j = 0; j < INTERLEAVER_FEPTH; j++){                    
            position = interleaver_matrix[j % 4][j / 4];
            //printf("position is %d\n",position);
            current_bit = read_dual_bytes >> (position-1) & 1;
            deinterlev_dual_bytes += current_bit << j;          
        }        
        deinterleaved_msg[2*i] = deinterlev_dual_bytes & 0xff;
        deinterleaved_msg[2*i+1] = (deinterlev_dual_bytes >> 8) & 0xff;
    }
}
