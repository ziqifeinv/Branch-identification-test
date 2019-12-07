
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "wq_vtb/ext_inc/wq_vtb_topo_rec.h"

#define READ_BUFFER_LEN	1000
#define ADC_DATA_LEN 7000

char data_str[] = "<-- wq_vtb_adc_data_format_print -->";
char read_buffer[READ_BUFFER_LEN] = { 0 };
int data[ADC_DATA_LEN] = { 0 };

static void iot_plc_hw_topo_data_print(uint8_t* buf,
	uint32_t len, uint8_t phase)
{
	printf("<--%s-- ", __FUNCTION__);
	for (uint32_t i = 0; i < len; ++i) {
		printf("%02x ", buf[i]);
	}
	printf("-->len %lu, phase %lu\n", len, phase);
}

int main(void)
{
	FILE* file = NULL;
	uint32_t data_len = 0;
	uint32_t index = 0;
	uint8_t	data_flag = 0;
	char temp_num[8] = { 0 };
	int temp_data = 0;
	uint32_t line_counter = 0;
	uint32_t i = 0;

	uint8_t complete = 0;
	uint8_t decode_data[20] = { 0 };
	uint8_t length = 0;
	int8_t offset = 0;
	int16_t data_offset = 0;
	uint32_t data_counter = 0;

	uint8_t *topo_handle = wq_vtb_topo_rx_create(4);
	if (topo_handle == NULL) {
		printf("topo create fail\n");
		exit(0);
	}
	file = fopen("≤‚ ‘ ˝æ›/err2.log", "r");
	if (file == NULL) {
		printf("file open fail!\n");
		getchar();
		return 0;
	}
	while (!feof(file)) {
		if (fgets(read_buffer, READ_BUFFER_LEN, file) != NULL) {
			//puts(read_buffer);
			if (memcmp(read_buffer, data_str, sizeof(data_str) - 1) == 0) {
				//printf("find data \n");
				data_flag = 1;
				memset(data, 0, sizeof(data));
				continue;
			}
			if (data_flag == 1) {
				line_counter++;
				//printf("start delete data,line:%d\n",line_counter);
				uint32_t old_i = 0;
				for (i = 0; i < READ_BUFFER_LEN; i++) {
					if (read_buffer[i] == ',') {
						memset(temp_num, 0, sizeof(temp_num));
						memcpy(temp_num, &read_buffer[old_i], i - old_i);
						temp_data = atoi(temp_num);
						data[index] = temp_data;
						index++;
						old_i = i + 1;
					}
					else if ((read_buffer[i] == '.' && read_buffer[i + 1] == '.') 
						|| read_buffer[i] == '\0'
						|| (read_buffer[i] == '[' && read_buffer[i+1] == 'D')) {
						break;
					}
				}
				if (line_counter == 101) {
					line_counter = 0;
					data_len = index;
					index = 0;
					data_flag = 2;
				}
			}
			else if (data_flag == 2) {
				data_flag = 0;
				data_counter++;
				complete = wq_vtb_topo_bit_rec(topo_handle, WQ_VTB_TOPO_RX_PHASE_A, data, data_len,
					8, decode_data, &length, &offset, &data_offset);
				if (complete) {
					iot_plc_hw_topo_data_print(decode_data, length, 0);
				}

				//complete = wq_vtb_topo_bit_rec(topo_handle, WQ_VTB_TOPO_RX_PHASE_B, data, data_len,
				//	8, decode_data, &length, &offset, &data_offset);
				//if (complete) {
				//	iot_plc_hw_topo_data_print(decode_data, length, 1);
				//}

				//complete = wq_vtb_topo_bit_rec(topo_handle, WQ_VTB_TOPO_RX_PHASE_C, data, data_len,
				//	8, decode_data, &length, &offset, &data_offset);
				//if (complete) {
				//	iot_plc_hw_topo_data_print(decode_data, length, 2);
				//}
				continue;
			}
		}
		else {
			printf("fgets err \n");
		}
	}

	fclose(file);
	printf("over!!!!\n");
	getchar();
	return 1;
}