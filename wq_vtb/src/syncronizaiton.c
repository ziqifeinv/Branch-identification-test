#include "syncronization.h"
#include "preamble_generator.h"
#include "portable.h"

uint32_t get_potential_preamble(uint8_t *demod_msg        ){
    uint32_t preamble = 0; uint8_t temp =0;
    uint16_t byte_index = 0; uint8_t bit_index_in_byte = 0;
    for(uint16_t i = 0; i < PREAMBLE_LENGTH; i++){
        byte_index = (i)/8;
        bit_index_in_byte = (i) % 8;
        temp = demod_msg[byte_index] >> bit_index_in_byte & 1;
        preamble += (temp << i);
    }
    return preamble;
}

uint32_t get_potential_preamble_topo(uint8_t *demod_msg        ){
    uint32_t preamble = 0; uint8_t temp =0;
    uint16_t byte_index = 0; uint8_t bit_index_in_byte = 0;
    for(uint16_t i = 0; i < PREAMBLE_LENGTH; i++){
        byte_index = (i)/8;
        bit_index_in_byte = (i) % 8;
        temp = demod_msg[byte_index] >> bit_index_in_byte & 1;
        preamble += (temp << i);
    }
    return preamble;
}

void update_preamble_register_left(uint8_t *dem_data,
                                        uint8_t current_data){
    uint8_t current_byte_index = 0, current_bit_index = 0, temp = 0;
    for(unsigned int i = (32 - 1) * 25 + 1; i > 0; i--){
        current_byte_index = (i - 1)/ 8 ; current_bit_index = (i-1) % 8;
        if(!current_bit_index){
            if(!current_byte_index){
                dem_data[current_byte_index] &= 0xfe;
                dem_data[current_byte_index] += current_data;
            }
            else{
                dem_data[current_byte_index] &= 0xfe;
                temp = dem_data[current_byte_index - 1] >> 7 & 1;
                dem_data[current_byte_index] += temp;
            }
        }
        else{
            dem_data[current_byte_index] &= (0xff - (1 << (current_bit_index)));
            temp = dem_data[current_byte_index] >> (current_bit_index - 1) & 1;
            dem_data[current_byte_index] += temp << current_bit_index;
        }
    }
}

void update_preamble_register_right(uint8_t *dem_data,
                                        uint8_t current_data){
    uint8_t current_byte_index = 0, current_bit_index = 0, temp = 0;
    for(unsigned int i = 0; i < 32; i++){
        current_byte_index = i / 8 ; current_bit_index = i % 8;
        if(current_bit_index == 7){
            if(current_byte_index == 3){
                dem_data[current_byte_index] &= 0x7f;
                dem_data[current_byte_index] += (current_data << 7);
            }
            else{
                dem_data[current_byte_index] &= 0x7f;
                temp = dem_data[current_byte_index + 1] & 1;
                dem_data[current_byte_index] += (temp << 7);
            }
        }
        else{
            dem_data[current_byte_index] &= (0xff - (1 << (current_bit_index)));
            temp = dem_data[current_byte_index] >> (current_bit_index + 1) & 1;
            dem_data[current_byte_index] += temp << current_bit_index;
        }
    }
}

void update_preamble_register_right_topo(uint8_t *dem_data,
                                        uint8_t current_data){
    uint8_t current_byte_index = 0, current_bit_index = 0, temp = 0;
    for(unsigned int i = 0; i < 32; i++){
        current_byte_index = i / 8 ; current_bit_index = i % 8;
        if(current_bit_index == 7){
            if(current_byte_index == 3){
                dem_data[current_byte_index] &= 0x7f;
                dem_data[current_byte_index] += (current_data << 7);
            }
            else{
                dem_data[current_byte_index] &= 0x7f;
                temp = dem_data[current_byte_index + 1] & 1;
                dem_data[current_byte_index] += (temp << 7);
            }
        }
        else{
            dem_data[current_byte_index] &= (0xff - (1 << (current_bit_index)));
            temp = dem_data[current_byte_index] >> (current_bit_index + 1) & 1;
            dem_data[current_byte_index] += temp << current_bit_index;
        }
    }
}

