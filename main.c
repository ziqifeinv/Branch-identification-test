
#define _CRT_SECURE_NO_WARNINGS

//common
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>
#include <time.h>

#include "wq_vtb/ext_inc/wq_vtb_topo_rec.h"
#include "wq_vtb/ext_inc/wq_vtb_bit_rec.h"

#define READ_BUFFER_LEN		1000
#define ADC_DATA_LEN		7000
#define ADC_DATA_LEN_LAST	261

/* define data file type */
#define FILE_TYPE_TSFM		0
#define FILE_TYPE_TOPO		1

#define DATA_FRAME_LEN_MAX  7

/* debug macro*/
#define USER_DEBUG			1

/* define error type*/
#define ERR_OK				0
#define ERR_FAIL			1

//globle
uint32_t g_data_counter = 0;
uint32_t g_line_counter_total = 0;
char g_read_buffer[READ_BUFFER_LEN] = { 0 };

FILE* g_log_file = NULL;

typedef struct _data_file_info_t {
	FILE* data_file;
	char* data_file_path;
	FILE* log_file;

	//data info
	//Type of raw data, 0->topo; 1->tsfm
	uint8_t type;
	//The corresponding algorithm version when capturing the original data
	uint32_t version;
	//Whether the original data is valid, 0 -> we should give up data
	uint8_t flag;
	//Length of valid data contained in the log
	uint8_t len;
	//Valid data contained in log
	uint8_t data[DATA_FRAME_LEN_MAX];
	//The number of valid data contained in the log
	uint8_t number;

	//decode info
	//1 -> The data was decrypted
	uint8_t decode_flag;
	//Length of valid data contained in the decode
	uint8_t decode_len;
	//Valid data contained in decode
	uint8_t decode_data[DATA_FRAME_LEN_MAX];
	//The number of valid data contained in the decode
	uint8_t decode_number;
} data_file_info_t;

void log_printf(const char* format, ...)
{
	if (g_log_file) {
		va_list args;
		va_start(args, format);
		//fprintf(g_log_file, format);
		vfprintf(g_log_file, format, args);
		va_end(args);
	}
}

void iot_plc_hw_topo_data_print(uint8_t* buf,
	uint32_t len, uint8_t phase)
{
	printf("<--%s-- ", __FUNCTION__);
	log_printf("<--%s-- ", __FUNCTION__);
	for (uint32_t i = 0; i < len; ++i) {
		printf("%02x ", buf[i]); 
		log_printf("%02x ", buf[i]);
	}
	printf("-->len %lu, phase %lu\n", len, phase);
	log_printf("-->len %lu, phase %lu\n", len, phase);
}

static void iot_plc_hw_tsfm_data_print(uint8_t* buf, uint32_t len,
	uint8_t flag)
{
	printf("<--%s-- ", __FUNCTION__);
	log_printf("<--%s-- ", __FUNCTION__);
	for (uint32_t i = 0; i < len; ++i) {
		printf("%02x ", buf[i]);
		log_printf("%02x ", buf[i]);
	}
	printf("-->len %lu, flag %lu\n", len, flag);
	log_printf("-->len %lu, flag %lu\n", len, flag);
}

void topo_data_handle(data_file_info_t* info)
{
	uint32_t data_len = 0;
	uint32_t index = 0;
	uint8_t	data_flag = 0;
	char temp_num[10] = { 0 };
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
		log_printf("topo create fail\n");
		exit(0);
	}

	while (!feof(info->data_file)) {
		if (fgets(g_read_buffer, READ_BUFFER_LEN, info->data_file) != NULL) {
			g_line_counter_total++;
			if (memcmp(g_read_buffer, topo_keyword, sizeof(topo_keyword) - 1) == 0) {
				data_flag = 1;
				memset(data, 0, sizeof(data));
				continue;
			}
			if (data_flag == 1) {
				line_counter++;
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
						if ((i - old_i) > 10) {
							old_i = i + 1;
						}
						else {
							memset(temp_num, 0, sizeof(char) * 10);
							memcpy(temp_num, &g_read_buffer[old_i], i - old_i);
							temp_data = atoi(temp_num);
							data[index] = temp_data;
							index++;
							old_i = i + 1;
						}
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
				if (info->version == 0x0300) { //3.0:only 25 cycles per data
					int* p_temp = malloc(sizeof(int) * data_len);
					if (p_temp == NULL) {
						printf("malloc err \n");
					}
					memcpy(p_temp, data, sizeof(int) * data_len);
					memcpy(data, data_last, sizeof(int) * ADC_DATA_LEN_LAST);
					memcpy(&data[ADC_DATA_LEN_LAST], p_temp, sizeof(int) * data_len);
					data_len += ADC_DATA_LEN_LAST;
					memcpy(data_last, &data[data_len - ADC_DATA_LEN_LAST], sizeof(int) * ADC_DATA_LEN_LAST);
					free(p_temp);
				}
				complete = wq_vtb_topo_bit_rec(topo_handle, WQ_VTB_TOPO_RX_PHASE_A, data, data_len,
					8, decode_data, &length, &offset, &data_offset, 0);
				if (complete) {
					//iot_plc_hw_topo_data_print(decode_data, length, 0);	//other space
					//now we think that the data in the log is the same
					if (!info->decode_flag) {
						info->decode_flag = 1;
						info->decode_len = length;
						info->decode_number = 1;
						memcpy(info->decode_data, decode_data, length);
					}else if (info->decode_len == length
						&& memcmp(info->decode_data, decode_data, length) == 0) {
						info->decode_number++;
					}
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
			printf("%s fgets err \n", __FUNCTION__);
			log_printf("%s fgets err \n", __FUNCTION__);
		}
	}
}

void tsfm_data_handle(data_file_info_t *info)
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
		log_printf("tsfm handle create failed \n");
		exit(0);
	}

	g_data_counter = 0;
	g_line_counter_total = 0;

	while (!feof(info->data_file)) {
		if (fgets(g_read_buffer, READ_BUFFER_LEN, info->data_file) != NULL) {
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
							//now we think that the data in the log is the same
							if (!info->decode_flag) {
								info->decode_flag = 1;
								info->decode_len = data_len;
								info->decode_number = 1;
								memcpy(info->decode_data, tsfm_decode_data, data_len);
							} else if (info->decode_len == data_len
								&& memcmp(info->decode_data, tsfm_decode_data, data_len) == 0) {
								info->decode_number++;
							}
							decode_data_counter++;
						}
						break;
					}
				}
			}

		}
		else {
			printf("%s fgets err \n", __FUNCTION__);
			log_printf("%s fgets err \n", __FUNCTION__);
		}
	}
}

#define SYNOPSIS_TYPE_NUMBER		8

static const char *g_synopsis_keyword[SYNOPSIS_TYPE_NUMBER] = {
	"synopsis start",	//0
	"data_type:",		//1
	"data_version:",	//2
	"data_flag:",		//3
	"data_len:",		//4
	"data:",			//5
	"data_number:",		//6
	"synopsis end and then there's the real data"	//7
};

static uint32_t synopsis_handle(data_file_info_t* info)
{
	uint32_t result = ERR_FAIL;
	uint32_t i = 0, j = 0;

	while (!feof(info->data_file)) {
		if (fgets(g_read_buffer, READ_BUFFER_LEN, info->data_file) != NULL) {
			g_line_counter_total++;
			puts(g_read_buffer);
			for (i = 0; i < SYNOPSIS_TYPE_NUMBER; i++) {
				if (memcmp(g_read_buffer, g_synopsis_keyword[i], strlen(g_synopsis_keyword[i])) == 0) {
					result = ERR_OK;
					break;
				}
			}
			switch (i)
			{
			case 0:
				break;
			case 1:
				if (memcmp(&g_read_buffer[strlen(g_synopsis_keyword[i])], "tsfm", 4) == 0) {
					info->type = 1;
				}
				else if (memcmp(&g_read_buffer[strlen(g_synopsis_keyword[i])], "topo", 4) == 0) {
					info->type = 0;
				}
				break;
			case 2:
				//info->version = atoi(&g_read_buffer[strlen(g_synopsis_keyword[i])]);
				(void)sscanf(&g_read_buffer[strlen(g_synopsis_keyword[i])], "%x", &info->version);
				break;
			case 3:
				info->flag = atoi(&g_read_buffer[strlen(g_synopsis_keyword[i])]);
				break;
			case 4:
				info->len = atoi(&g_read_buffer[strlen(g_synopsis_keyword[i])]);
				//if (info->len <= 0) {
				//	result = ERR_FAIL;
				//}
				break;
			case 5:
				(void)sscanf(&g_read_buffer[strlen(g_synopsis_keyword[i])], "%x %x %x %x %x %x %x",
					&info->data[0], &info->data[1], &info->data[2], &info->data[3],
					&info->data[4], &info->data[5], &info->data[6]);
				break;
			case 6:
				info->number = atoi(&g_read_buffer[strlen(g_synopsis_keyword[i])]);
				break;
			case (SYNOPSIS_TYPE_NUMBER - 1):
				goto out;
				break;
			default:
				break;
			}
			//if ((result != ERR_OK) || (i == SYNOPSIS_TYPE_NUMBER - 1)) {
			//	goto out;
			//}
		}
		else {
			printf("%s fgets err \n", __FUNCTION__);
			log_printf("%s fgets err \n",__FUNCTION__);
		}
	}
out:
	if (result != ERR_OK) {
		printf("no header info\n");
	}
	return result;
}

static void result_reduction(data_file_info_t *info)
{
	int result = ERR_FAIL;
	uint32_t i = 0;

	printf("[summarize]: %s\n", info->data_file_path);
	log_printf("[summarize]: %s\n", info->data_file_path);
	printf("data file info:   data flag:%d, data len:%d, data num:%d, data:",
		info->flag, info->len, info->number);
	log_printf("data file info:   data flag:%d, data len:%d, data num:%d, data:",
		info->flag, info->len, info->number);
	for (i = 0; i < info->len; i++) {
		printf("0x%02X ", info->data[i]);
		log_printf("0x%02X ", info->data[i]);
	}
	printf("\n");
	log_printf("\n");

	printf("data decode info: data flag:%d, data len:%d, data num:%d, data:",
		info->decode_flag, info->decode_len, info->decode_number);
	log_printf("data decode info: data flag:%d, data len:%d, data num:%d, data:",
		info->decode_flag, info->decode_len, info->decode_number);
	for (i = 0; i < info->decode_len; i++) {
		printf("0x%02X ", info->decode_data[i]);
		log_printf("0x%02X ", info->decode_data[i]);
	}
	printf("\n");
	log_printf("\n");

	result = memcmp(info->data, info->decode_data, info->len);
	if ((info->len == info->decode_len)
		&& (info->flag == info->decode_flag)
		&& (info->number == info->decode_number)
		&& (result == 0)) {
		result == ERR_OK;
	} else {
		result = ERR_FAIL;
	}

	printf("decode %s \n", (result ? "failed" : "secceed"));
	log_printf("decode %s \n", (result ? "failed" : "secceed"));
	return;
}

int main(int argc, char *argv[])
{
	uint32_t result = 0;
	data_file_info_t* info = NULL;

	time_t raw_time = 0;
	struct tm* tm_info = NULL;
	char buffer[80] = { 0 };

	//check argv
	//printf("argc:%d\n", argc);
	//for (uint8_t i = 0; i < argc; i++) {
	//	printf("argv[%d]:%s \n", i, argv[i]);
	//}
	//printf("\n");
#if !USER_DEBUG
	if (argc < 2) {
		printf("please enter the data file path\n");
		log_printf("please enter the data file path\n");
		result = 1;
		goto out;
	}
#endif
	
	//create file info handle
	info = malloc(sizeof(data_file_info_t));
	if (info == NULL) {
		printf("create info handle failed \n");
		log_printf("create info handle failed \n");
		result = 2;
		goto out;
	}
	memset(info, 0, sizeof(data_file_info_t));

	//create log file
	time(&raw_time);
	tm_info = localtime(&raw_time);
	sprintf(buffer, "%d-%02d-%02d %02d-%02d-%02d.log", (tm_info->tm_year + 1900), (tm_info->tm_mon + 1),
		tm_info->tm_mday, tm_info->tm_hour, tm_info->tm_min, tm_info->tm_sec);
	printf("the log file name generated by this test is:%s \n", buffer);
	log_printf("the log file name generated by this test is:%s \n", buffer);
	info->log_file = fopen(buffer, "w");
	if (info->log_file == NULL) {
		printf("create log file failed! \n");
		log_printf("create log file failed! \n");
		result = 3;
		goto out;
	}
	g_log_file = info->log_file;

	//open data file 
#if USER_DEBUG
	info->data_file_path = "D:\\workspace\\vs\\file_load\\test_data\\topo 3.0\\8_topo3.0.log";
	info->data_file = fopen(info->data_file_path, "r");
#else
	infi->data_file_path = argv[1];
	info->data_file = fopen(info->data_file_path, "r");
#endif
	if (info->data_file == NULL) {
		printf("data file open failed, path:%s \n", info->data_file_path);
		log_printf("data file open failed, path:%s \n", info->data_file_path);
		result = 4;
		goto out;
	}
	printf("data file open succeed, path:%s \n", info->data_file_path);
	log_printf("data file open succeed, path:%s \n", info->data_file_path);

	//get data file information
	result = synopsis_handle(info);
	if (result != ERR_OK) {
		result = 5;
		goto out;
	}

	if (info->type) {
		tsfm_data_handle(info);
	}
	else {
		topo_data_handle(info);
	}

	result_reduction(info);
	printf("over!!!!\n");
	log_printf("over!!!!\n");
	result = 0;

out:
	if (info && info->data_file) {
		fclose(info->data_file);
	}

	if (info && info->log_file) {
		fclose(info->log_file);
	}
	(void)getchar();
	return result;
}