#include "preamble_generator.h"
#include "bit.h"

void preamble_prepend(uint8_t *msg,
                         uint8_t msg_len){
    unsigned long *preamble_vector = preuso_noise_sequence();
    unsigned long preamble = *(preamble_vector + (msg_len/2 - 3)- 1);
    //printf("preamble is %4x\n",preamble);
    uint8_t preamble_pointer[4];
    //if(!preamble_pointer){
        //printf("preamble memory allocation failed!\n");
    //}
    for(uint8_t i = 0; i < PREAMBLE_BYTES; i++){
        preamble_pointer[i] = (preamble >> (8 * i)) & 0xff;
        //printf("preamble %4x\n",preamble_pointer[i]);
    }
    for(uint8_t i = msg_len + PREAMBLE_BYTES - 1; i > PREAMBLE_BYTES - 1 ; i--){
        msg[i] = msg[i-PREAMBLE_BYTES];
    }
    for(uint8_t i = 0; i < PREAMBLE_BYTES; i++){
        msg[i] = preamble_pointer[i];
    }
}

unsigned long *preuso_noise_sequence(        ){
    static unsigned long preamble_vec[7];
    unsigned long register_state[7] =
        {0x1a1, 0x13b, 0x1f2, 0x18b, 0x141, 0x11f, 0x1ff};
    for(uint8_t j = 0; j < 7; j++){
        unsigned long preuso_noise_seq = 0, temp = 0;
        for(uint8_t i = 0; i < PREAMBLE_LENGTH; i++){
            temp = (register_state[j] >> 3 & 1)
                ^ (register_state[j] >> (REGISTER_ORDER - 1) & 1);
            //printf("temp is :%d\n",temp);
            register_state[j] = (register_state[j] << 1 & 0x1ff) ^ temp;
            //printf("state is %4x\n",register_state);
            preuso_noise_seq += temp << i;
            //printf("seq is %4x\n",preuso_noise_seq);
        }
        //printf("seq is %4x\n",preuso_noise_seq);
        preamble_vec[j] = preuso_noise_seq;
    }
    return preamble_vec;
}

unsigned long *preuso_noise_sequence_topo(       ){
    static unsigned long preamble_vec_topo[PREAMBLE_TYPE_COUNT];
    unsigned long register_state[PREAMBLE_TYPE_COUNT] =
        {PREAMBLE_TYPE_1_BYTE, PREAMBLE_TYPE_3_BYTE};
    for(uint8_t j = 0; j < PREAMBLE_TYPE_COUNT; j++){
        unsigned long preuso_noise_seq = 0, temp = 0;
        for(uint8_t i = 0; i < PREAMBLE_LENGTH; i++){
            temp = (register_state[j] >> 3 & 1)
                ^ (register_state[j] >> (REGISTER_ORDER - 1) & 1);
            //printf("temp is :%d\n",temp);
            register_state[j] = (register_state[j] << 1 & 0x1ff) ^ temp;
            //printf("state is %4x\n",register_state);
            //preuso_noise_seq += temp << (PREAMBLE_LENGTH - 1 - i);
            preuso_noise_seq += temp << i;
            //printf("seq is %4x\n",preuso_noise_seq);
        }
        //printf("seq is %4x\n",preuso_noise_seq);
        preamble_vec_topo[j] = preuso_noise_seq;
    }
    return preamble_vec_topo;
}

