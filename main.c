
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "wq_vtb/ext_inc/wq_vtb_topo_rec.h"
#include "wq_vtb/ext_inc/wq_vtb_bit_rec.h"
#include "wq_vtb/inc/wq_version.h"

#define READ_BUFFER_LEN	1000
#define ADC_DATA_LEN 7000
#define ADC_DATA_LEN_LAST	261

//globle
uint32_t g_data_counter = 0;
uint32_t g_line_counter_total = 0;
char g_read_buffer[READ_BUFFER_LEN] = { 0 };

static void iot_plc_hw_topo_data_print(uint8_t* buf,
	uint32_t len, uint8_t phase)
{
	printf("<--%s-- ", __FUNCTION__);
	for (uint32_t i = 0; i < len; ++i) {
		printf("%02x ", buf[i]);
	}
	printf("-->len %lu, phase %lu\n", len, phase);
}

static void iot_plc_hw_tsfm_data_print(uint8_t* buf, uint32_t len,
	uint8_t flag)
{
	printf("<--%s-- ", __FUNCTION__);
	for (uint32_t i = 0; i < len; ++i) {
		printf("%02x ", buf[i]);
	}
	printf("-->len %lu, flag %lu\n", len, flag);
}

void topo_data_handle(FILE *file)
{
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
	uint8_t data_err = 0;
	char topo_keyword[] = "<-- wq_vtb_adc_data_format_print";
	int data[ADC_DATA_LEN] = { 0 };
	int data_last[ADC_DATA_LEN_LAST] = { 0 };

	uint8_t* topo_handle = wq_vtb_topo_rx_create(4);
	if (topo_handle == NULL) {
		printf("topo create fail\n");
		exit(0);
	}

	while (!feof(file)) {
		if (fgets(g_read_buffer, READ_BUFFER_LEN, file) != NULL) {
			g_line_counter_total++;
			//printf("read data:");
			//puts(g_read_buffer);


			if (memcmp(g_read_buffer, topo_keyword, sizeof(topo_keyword) - 1) == 0) {
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
					if ((g_read_buffer[i] == '[' && g_read_buffer[i + 1] == 'D')) {
						data_err++;
					}
					else if ((g_read_buffer[i] == '.' && g_read_buffer[i + 1] == '.')
						|| g_read_buffer[i] == '\0') {
						break;
					}
					else if ((g_read_buffer[i] == ',')) {
						memset(temp_num, 0, sizeof(temp_num));
						memcpy(temp_num, &g_read_buffer[old_i], i - old_i);
						temp_data = atoi(temp_num);
						data[index] = temp_data;
						index++;
						old_i = i + 1;
					}
				}
				if (line_counter == 105 || data_err >= 2) {
					data_err = 0;
					line_counter = 0;
					data_len = index;
					index = 0;
					data_flag = 2;
				}
			}
			if (data_flag == 2) {
				data_flag = 0;
				g_data_counter++;
				//这里是为了兼容老的log，将n-1次最后261个数据放在n次数据的开始位置
				//int* p_temp = malloc(sizeof(int) * data_len);
				//if (p_temp == NULL) {
				//	printf("malloc err \n");
				//}
				//memcpy(p_temp, data, sizeof(int) * data_len);
				//memcpy(data, data_last, sizeof(int) * ADC_DATA_LEN_LAST);
				//memcpy(&data[ADC_DATA_LEN_LAST], p_temp, sizeof(int) * data_len);
				//data_len += ADC_DATA_LEN_LAST;
				//memcpy(data_last, &data[data_len - ADC_DATA_LEN_LAST], sizeof(int) * ADC_DATA_LEN_LAST);
				//free(p_temp);

				complete = wq_vtb_topo_bit_rec(topo_handle, WQ_VTB_TOPO_RX_PHASE_A, data, data_len,
					8, decode_data, &length, &offset, &data_offset, 0);
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
}

void tsfm_data_handle(FILE* file)
{
	/*iot_plc_hw_tsfm_gpio_task_handler delta 19993*/
	uint8_t data_len = 0;
	uint8_t complete = 0;
	uint8_t* tsfm_handler = NULL;
	uint32_t delta = 0;
	char tsfm_keyword[] = "iot_plc_hw_tsfm_gpio_task_handler delta ";
	uint8_t tsfm_decode_data[30] = { 0 };
	uint32_t i = 0;
	char delta_temp[5] = { 0 };
	uint32_t decode_data_counter = 0;

	tsfm_handler = wq_vtb_rx_create();
	if (tsfm_handler == NULL) {
		printf("tsfm handle create failed \n");
		exit(0);
	}

	g_data_counter = 0;
	g_line_counter_total = 0;

	while (!feof(file)) {
		if (fgets(g_read_buffer, READ_BUFFER_LEN, file) != NULL) {
			g_line_counter_total++;
			if (memcmp(g_read_buffer, tsfm_keyword, sizeof(tsfm_keyword) - 1) == 0) {
				g_data_counter++;
				for (i = 0; i < READ_BUFFER_LEN; i++) {
					if (g_read_buffer[i] == 'd' && g_read_buffer[i + 1] == 'e'
						&& g_read_buffer[i + 2] == 'l' && g_read_buffer[i + 3] == 't') {
						memset(delta_temp, 0, sizeof(char) * 5);
						memcpy(delta_temp, &g_read_buffer[i + 6], sizeof(char) * 5);
						delta = atoi(delta_temp);
						complete = wq_vtb_bit_rec(tsfm_handler, delta, tsfm_decode_data, &data_len);
						if (complete) {
							iot_plc_hw_tsfm_data_print(tsfm_decode_data, data_len, 3);
							decode_data_counter++;
						}
						break;
					}
				}
			}

		}
		else {
			printf("fgets err \n");
		}
	}
	printf("tsfm data found %d times \n", decode_data_counter);
}

int main(void)
{
	//freopen("vs_log.log", "w", stdout);

	FILE* file = NULL;
	uint8_t data_type = 0;

	//	normol-delta300
	//	5850正常识别-delta300
	//	5439未识别-delta60
	file = fopen("测试数据/tsfm/5439未识别-delta60", "r");
	if (file == NULL) {
		printf("file open fail!\n");
		getchar();
		return 0;
	}
	
	//topo_data_handle(file);
	tsfm_data_handle(file);

	fclose(file);
	printf("over!!!!\n");
	//getchar();
	return 1;
}