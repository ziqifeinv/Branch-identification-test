#include "disturbance_detection.h"
#include "wq_debug.h"
#include <stdlib.h>

#define VS_DATA_LEN 7000

//void wq_vtb_adc_data_format_print(int32_t* buf, uint32_t len, uint8_t phase)
//{
//    static uint32_t pack_cnt = 1;
//
//    iot_wq_printf("<-- %s --> idx %d, len %lu, phase %lu\n",
//        __FUNCTION__, pack_cnt++, len, phase);
//
//    for (uint32_t i = 0; i < len; ++i) {
//        iot_wq_printf("%d,", buf[i]);
//        if (((i + 1) % 64) == 0 && i != 0) {
//            iot_wq_printf("...\n");
//        }
//    }
//
//    iot_wq_printf("...\n");
//
//    return;
//}

void get_pos_max_magnitude(float *data, uint16_t *group_pos, uint8_t num, float *max_val){
    for(uint8_t i = 0; i<num; i++){
        if(abs(data[group_pos[i]]) > *max_val){
            *max_val = abs(data[group_pos[i]]);
        }
    }
}

float get_min(float *val, uint8_t len, uint8_t *idx){
    float min_val = val[0];
    for(uint16_t i = 0; i<len; i++){
        if(val[i]<min_val){
            *idx = i;
            min_val = val[i];
        }
    }
    return min_val;
}

//sort the position, and adjust its magnitude accordingly
void sort(uint16_t *arr, float *val, uint8_t size){
    uint8_t i, j;
    uint16_t tmp;
    float v;
    for (i = 0; i < size - 1; i++) {
        for (j = 0; j < size - i - 1; j++) {
            if (arr[j] > arr[j+1]) {
                tmp = arr[j]; v = val[j];
                arr[j] = arr[j+1]; val[j] = val[j+1];
                arr[j+1] = tmp; val[j+1] = v;
            }
        }
    }
}

uint8_t check_sign(float *data, uint16_t *current_group_pos, uint8_t num, uint16_t ref_pos){
    uint8_t cnt = 0, tot_cnt = 0;
    for(uint8_t i=0;i<num;i++){
        if(current_group_pos[i]-ref_pos <=10) {
            tot_cnt++;
            if(data[current_group_pos[i]] > 0){
                cnt++;
            }
        }
    }
    wq_dbg_printf("cnt %d, tol %d.", cnt, tot_cnt);
    if((cnt>=5)||(cnt == tot_cnt)){
        return 1;
    }
    return 0;
}

uint8_t check_sign_new(float *data, uint16_t *current_group_pos, uint8_t num, uint16_t ref_pos){
//    uint16_t up_pos[num], low_pos[num], first_half[num];
	uint16_t* up_pos = (uint16_t*)malloc(sizeof(uint16_t) * num);
	uint16_t* low_pos = (uint16_t*)malloc(sizeof(uint16_t) * num);
	uint16_t* first_half = (uint16_t*)malloc(sizeof(uint16_t) * num);
    memset(up_pos, 0, sizeof(up_pos));
    memset(low_pos, 0, sizeof(low_pos));
    memset(first_half, 0, sizeof(first_half));
    uint8_t up_num = 0, low_num = 0, i, tot_cnt = 0;
	
    for(i=0;i<num;i++){
        if(current_group_pos[i]-ref_pos < 64) {
            first_half[tot_cnt] = current_group_pos[i];
            tot_cnt++;
        }
    }

    float max_mag = 0, min_mag = 0;

    for(i= 0;i<tot_cnt;i++){
        if(data[first_half[i]]>0){
            up_num++;
        }
        else{
            low_num++;
        }
        if(data[first_half[i]]>max_mag){
            max_mag = data[first_half[i]];
        }
        if(data[first_half[i]] < min_mag){
            min_mag = data[first_half[i]];
        }
    }

    wq_dbg_printf("positive number %d, negative number %d.", up_num, low_num);
    if((up_num > low_num)||(max_mag > abs(min_mag))){
        return 1;
    }
	free(up_pos);
	free(low_pos);
	free(first_half);
    return 0;
}

uint8_t check_sign_new1(float *data, uint16_t data_len, uint16_t ref_pos){
    uint16_t start_pos, end_pos;
    start_pos = (ref_pos < 138) ? 0 : (ref_pos-138);
    end_pos = (ref_pos > (data_len - 139)) ? (data_len - 1) : (ref_pos + 138);

    float temp_max = 0, temp_min = 0;
    uint16_t max_pos = 0, min_pos = 0;
    for(uint16_t i = start_pos; i<end_pos+1;i++){
        if(data[i]>temp_max){
            temp_max = data[i];
            max_pos = i;
        }
        if(data[i]<temp_min){
            temp_min = data[i];
            min_pos = i;
        }
    }
    if(min_pos < max_pos){
        return 1;
    }
    else{
        return 0;
    }
}

void convolution(int32_t *data, float *core, float *cov,
    uint16_t n, uint8_t m)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint16_t k = 0;

    for (i = 0; i < n; i++) {
        cov[i] = 0;
    }

    i = 0;

    for (j = 0; j < m / 2; j++, i++) {
        for (k = m / 2-j; k < m; k++ ) {
            cov[i] += data[k-(m / 2-j)] * core[k];
        }

        for (k = n-m / 2+j; k < n; k++ ) {
            cov[i] += data[k] * core[k-(n-m / 2 + j)];
        }
    }

    for ( i = m / 2; i <= (n - m) + m / 2; i++) {
        for ( j = 0; j < m; j++) {
            if (j == 0) {
                cov[i] += data[i-m / 2 + j];
            } else {
                cov[i] -= data[i-m / 2 + j];
            }
        }
    }

    for ( i = m / 2; i <= (n-m)+m / 2; i++) {
        for( j = 0; j < m; j++){
            cov[i] += data[i-m / 2 + j] * core[j];
        }
    }

    i = (n - m) + m/2 + 1;
    for (j = 1; j < m / 2; j++, i++) {
        for (k = 0; k < j; k++) {
            cov[i] += data[k] * core[m - j - k];
        }

        for (k = 0; k < m-j; k++) {
            cov[i] += core[k] * data[n-( m - j) + k];
        }
    }
}

void dwt(int32_t *input, float *output, float *temp, float *h,
    float *g, uint16_t n, uint8_t m)
{
    uint16_t i = 0;

    convolution(input, g, temp, n, m);

    for (i = 0; i < n / 2; i++) {
        output[i] = temp[i];//wave factor
    }
}

float math_sqrt_newton_iterative(float x)
{
	union {
		float x;
		int i;
	} u;
	u.x = x;
	u.i = 0x5f3759df - (u.i >> 1);
	return u.x * (1.5f - 0.5f * x * u.x * u.x) * x;
}

uint8_t statis_decision(uint32_t phase, float *data, uint16_t data_length, uint8_t sample_order,
    int32_t expect_pos, float ave_mag, wq_prm_bit_val_t *preamble_bits_data,int32_t *sample_data)
{
    float sum = 0.0;
    float ave = 0.0;
    float temp_dis = 0.0;
    uint16_t len = 1 << (sample_order-1);
    uint16_t i;

    //calculate mean of data
    for (i = 0; i < data_length; i++) {
        sum += data[i];
    }
    ave = sum/(data_length);
    // calculate variance of data
    for (i = 0; i < data_length; i++ ) {
        float dis = ave - data[i];
        temp_dis += dis * dis;
    }
    float var = math_sqrt_newton_iterative(temp_dis / (data_length - 1));
    uint16_t temp_num = 0, temp_pos[512];

    float coeff = 2;

    for (i = 0; i < data_length; i++ ) {
        if((data[i] > ave + coeff * var) || (data[i] < ave - coeff * var)){
            temp_pos[temp_num] = i;//positions have been sorted
            temp_num++; /* TODO : overflow */
        }
    }

    //sort(temp_pos, temp_num);

    wq_dbg_printf("possible found samples is %d\n",temp_num);

    //initialized the output struct
    preamble_bits_data->bit_cnt = 0;
    memset(&preamble_bits_data->bit_pos[0], 0, sizeof(uint16_t) * PREAMBLE_BIT_GROUP);
    memset(&preamble_bits_data->mag_sign[0], 0, sizeof(uint8_t) * PREAMBLE_BIT_GROUP);
    memset(&preamble_bits_data->mag_val[0], 0, sizeof(float) * PREAMBLE_BIT_GROUP);

    if(expect_pos < 0){ //if there is no expect position, certain possible positions need to return
        if(temp_num < 3){ //if not find any possible position, return directly
            preamble_bits_data->bit_cnt = 0;
            return 0;
        }

        //reference position, initialized with first position
        uint16_t ref_pos = temp_pos[0];
        uint16_t current_group_pos[138], group_pos_left[25];
        uint8_t sign_group_pos[25];
        float max_mag[25];
        //initialized the new variables
        memset(current_group_pos,0,sizeof(current_group_pos));
        memset(group_pos_left,0,sizeof(group_pos_left));
        memset(max_mag,0,sizeof(max_mag));
        memset(sign_group_pos,0,sizeof(sign_group_pos));
        uint8_t group_num = 1, count = 0;

        for(i=0; i < temp_num; i++){
            //if the select position is near by reference position, put them into one group
             if(temp_pos[i] - ref_pos < len + 10){
                current_group_pos[count] = temp_pos[i];
                count++;//number of position in current group
             }
             else{
                 //deal with current group: get the max magnitude and its position
                 get_pos_max_magnitude(data, current_group_pos, count, &max_mag[group_num-1]);
                 //save the reference position
                group_pos_left[group_num-1] = ref_pos;
                //check the sign of current group
                //sign_group_pos[group_num-1] = check_sign_new(data, current_group_pos, count, ref_pos);
                sign_group_pos[group_num-1] = check_sign_new1(data, data_length, ref_pos);
                //reset current position group
                memset(current_group_pos,0,sizeof(current_group_pos));
                //reset count
                count = 0;
                //put position into the next group (always the first one)
                current_group_pos[count] = temp_pos[i];
                count++;
                //update the reference position
                ref_pos = temp_pos[i];
                //increase the number of group
                 group_num++;
             }
        }

        wq_dbg_printf("find group number is %d\n",group_num);

        //deal with the last group: get the max magnitude and its position
        get_pos_max_magnitude(data, current_group_pos, count, &max_mag[group_num-1]);
        group_pos_left[group_num-1] = ref_pos;
        //sign_group_pos[group_num-1] = check_sign_new(data, current_group_pos, count, ref_pos);
        sign_group_pos[group_num-1] = check_sign_new1(data, data_length, ref_pos);
        //if the found number of groups is less than expectation, return directly
        if(group_num <= PREAMBLE_BIT_GROUP){
            preamble_bits_data->bit_cnt = group_num;
            memcpy(&preamble_bits_data->bit_pos[0], group_pos_left, sizeof(uint16_t) * group_num);
            memcpy(&preamble_bits_data->mag_sign[0], sign_group_pos, sizeof(uint8_t) * group_num);
            memcpy(&preamble_bits_data->mag_val[0], max_mag, sizeof(float) * group_num);
            return 0;
        }

        //otherwise, return the top 'PREAMBLE_BIT_GROUP' ones
        float value_pos[PREAMBLE_BIT_GROUP];
        uint16_t pos[PREAMBLE_BIT_GROUP];
        uint8_t sign_temp[PREAMBLE_BIT_GROUP];
        memset(value_pos, 0, sizeof(value_pos));
        memset(pos, 0, sizeof(pos));
        memset(sign_temp, 0, sizeof(sign_temp));
        uint8_t idx = 0;

        //select the top PREAMBLE_BIT_GROUP groups
        for(i = 0; i<group_num; i++){
            float min = get_min(value_pos, PREAMBLE_BIT_GROUP, &idx);
            if(max_mag[i] > min){
                pos[idx] = group_pos_left[i];
                value_pos[idx] = max_mag[i];
            }
        }
        //sort the select group position
        sort(pos, value_pos, PREAMBLE_BIT_GROUP);
        //search the sign of the selected group
        for(i = 0;i<PREAMBLE_BIT_GROUP;i++){
            for(uint8_t j = 0;j<group_num;j++){
                if(pos[i] == group_pos_left[j]){
                    sign_temp[i] = sign_group_pos[j];
                }
            }
        }
        preamble_bits_data->bit_cnt = PREAMBLE_BIT_GROUP;
        memcpy(&preamble_bits_data->bit_pos[0], pos, sizeof(uint16_t) * PREAMBLE_BIT_GROUP);
        memcpy(&preamble_bits_data->mag_sign[0], sign_temp, sizeof(uint8_t) * PREAMBLE_BIT_GROUP);
        memcpy(&preamble_bits_data->mag_val[0], value_pos, sizeof(float) * PREAMBLE_BIT_GROUP);
        return 0;
    }
    else{//if there exist an expect position, return demod bit according to the results around the expect position

        //need to delete later!!!!!!!!!!!!!!!!!!!!!!!
        //reference position, initialized with first position
        uint16_t ref_pos = temp_pos[0];
        uint16_t current_group_pos[138], group_pos_left[25];
        uint8_t sign_group_pos[25];
        float max_mag[25];
        memset(current_group_pos,0,sizeof(current_group_pos));
        memset(group_pos_left,0,sizeof(group_pos_left));
        memset(max_mag,0,sizeof(max_mag));
        memset(sign_group_pos,0,sizeof(sign_group_pos));
        uint8_t group_num = 1, count = 0, ret = 0;

        for(i=0; i < temp_num; i++){
            //if the select position is near by reference position, put them into one group
             if(temp_pos[i] - ref_pos < len + 10){
                current_group_pos[count] = temp_pos[i];
                count++;//number of position in current group
             }
             else{
                 //deal with current group: get the max magnitude and its position
                 get_pos_max_magnitude(data, current_group_pos, count, &max_mag[group_num-1]);
                group_pos_left[group_num-1] = ref_pos;
                //sign_group_pos[group_num-1] = check_sign_new(data, current_group_pos, count, ref_pos);
                sign_group_pos[group_num-1] = check_sign_new1(data, data_length, ref_pos);
                //reset current position group
                memset(current_group_pos,0,sizeof(current_group_pos));
                //reset count
                count = 0;
                //put position into the next group (always the first one)
                current_group_pos[count] = temp_pos[i];
                count++;
                //update the reference position
                ref_pos = temp_pos[i];
                //increase the number of group
                 group_num++;
             }
        }

        wq_dbg_printf("find group number is %d\n",group_num);

        //deal with the last group: get the max magnitude and its position
        get_pos_max_magnitude(data, current_group_pos, count, &max_mag[group_num-1]);
        group_pos_left[group_num-1] = ref_pos;
        //sign_group_pos[group_num-1] = check_sign_new(data, current_group_pos, count, ref_pos);
        sign_group_pos[group_num-1] = check_sign_new1(data, data_length, ref_pos);
        //if the found number of groups is less than expectation, return directly
        if(group_num <= PREAMBLE_BIT_GROUP){
            preamble_bits_data->bit_cnt = group_num;
            memcpy(&preamble_bits_data->bit_pos[0], group_pos_left, sizeof(uint16_t) * group_num);
            memcpy(&preamble_bits_data->mag_sign[0], sign_group_pos, sizeof(uint8_t) * group_num);
            memcpy(&preamble_bits_data->mag_val[0], max_mag, sizeof(float) * group_num);
            uint16_t find_sample = 0; float max_mag_expect_pos = 0.0;
            for(i=0; i < temp_num; i++){
                if(abs(temp_pos[i] - expect_pos) < (len)){
                    find_sample++;
                    if(abs(data[temp_pos[i]]) > max_mag_expect_pos){
                        max_mag_expect_pos = abs(data[temp_pos[i]]);
                    }
                }
            }

            if ((find_sample>=5) && (3 * max_mag_expect_pos >= ave_mag)) {
                ret = 1;
//                wq_dbg_printf("case 1");
//                wq_vtb_adc_data_format_print(sample_data,data_length *2,phase);
            }
            wq_dbg_printf("wave len %d, average magnitude %d, current group max magnitude %d.",
                find_sample, (int)ave_mag, (int)max_mag_expect_pos);
            return ret;
        }

        //otherwise, return the top 'PREAMBLE_BIT_GROUP' ones
        float value_pos[PREAMBLE_BIT_GROUP];
        uint16_t pos[PREAMBLE_BIT_GROUP];
        uint8_t sign_temp[PREAMBLE_BIT_GROUP];
        memset(value_pos, 0, sizeof(value_pos));
        memset(pos, 0, sizeof(pos));
        memset(sign_temp, 0, sizeof(sign_temp));
        uint8_t idx = 0;

        for(i = 0; i<group_num; i++){
            float min = get_min(value_pos, PREAMBLE_BIT_GROUP, &idx);
            if(max_mag[i] > min){
                pos[idx] = group_pos_left[i];
                value_pos[idx] = max_mag[i];
            }
        }

        sort(pos, value_pos, PREAMBLE_BIT_GROUP);
        for(i = 0;i<PREAMBLE_BIT_GROUP;i++){
            for(uint8_t j = 0;j<group_num;j++){
                if(pos[i] == group_pos_left[j]){
                    sign_temp[i] = sign_group_pos[j];
                }
            }
        }
        preamble_bits_data->bit_cnt = PREAMBLE_BIT_GROUP;
        memcpy(&preamble_bits_data->bit_pos[0], pos, sizeof(uint16_t) * PREAMBLE_BIT_GROUP);
        memcpy(&preamble_bits_data->mag_sign[0], sign_temp, sizeof(uint8_t) * PREAMBLE_BIT_GROUP);
        memcpy(&preamble_bits_data->mag_val[0], value_pos, sizeof(float) * PREAMBLE_BIT_GROUP);
        //need to delete later!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        uint16_t find_sample = 0; float max_mag_expect_pos = 0.0;
        for(i=0; i < temp_num; i++){
            if(abs(temp_pos[i] - expect_pos) < (len)){
                find_sample++;
                if(abs(data[temp_pos[i]]) > max_mag_expect_pos){
                    max_mag_expect_pos = abs(data[temp_pos[i]]);
                }
            }
        }
        if ((find_sample>=5) && (3 * max_mag_expect_pos >= ave_mag)) {
//            wq_dbg_printf("case 2");
//            wq_vtb_adc_data_format_print(sample_data,data_length *2,phase);
            ret = 1;
        }
        wq_dbg_printf("wave len %d, average magnitude %d, current group max magnitude %d.", find_sample, (int)ave_mag, (int)max_mag_expect_pos);
        return ret;
    }
}

void sum_move(int32_t *sample_data, uint16_t data_len,
    uint16_t moveLen, int32_t *res, int32_t *tail)
{
//    int32_t buff[moveLen];
	int32_t* buff = (int32_t*)malloc(sizeof(int32_t) * moveLen);
    uint16_t idx = 0;
    int32_t sum = 0;

    for (uint16_t i = 0; i < moveLen; i++) {
        buff[i] = tail[i];
        sum += buff[i];
    }

    for (uint16_t j = 0; j < data_len; j++) {
        idx = j % (moveLen);
        sum += sample_data[j] - buff[idx];
        buff[idx] = sample_data[j];
        res[j] = sum;
    }
	free(buff);
}

uint8_t disturb_detection(uint32_t phase, int32_t *sample_data, uint16_t data_len,
    uint8_t sample_order, int32_t exp_pos, float ave_mag, wq_prm_bit_val_t *preamble_bits_data)
{
    //float wave_analysis[VS_DATA_LEN];
    //float temp[VS_DATA_LEN / 2];
	float* wave_analysis = (float*)malloc(sizeof(float) * data_len);
	float* temp = (float*)malloc(sizeof(float) * (data_len));
    static float h[] = {0.7071, 0.7071};
    static float g[] = {-0.7071, 0.7071};
    uint8_t interval = 10;
    uint8_t wave_len = 2;
    uint8_t mask = 0;

    for (uint16_t i = 0; i<data_len;i++) {
        if (abs(sample_data[i]) > 50000) {
            if (i < interval) {
                continue;
            }
            if (i < ((1<<sample_order) + (interval<<1) + 1)) {
                for (uint8_t j = 0; j <= interval; j++) {
                    sample_data[i - (interval << 1) + j] =
                        sample_data[i + (1<<sample_order) - (interval<<1) + j];
                }
            } else {
                for (uint8_t j = 0; j <= interval; j++) {
                    sample_data[ i- (interval <<1 ) + j] =
                        sample_data[i - (1<<sample_order) - (interval<<1) + j];
                }
            }
        }
    }
    static int32_t tail5[5] = {0}, tail256[256] = {0}; /* TODO : phase. */
    //int32_t ave_result1[VS_DATA_LEN];
    //int32_t ave_result2[VS_DATA_LEN];
	int32_t* ave_result1 = (int32_t*)malloc(sizeof(int32_t) * data_len);
	int32_t* ave_result2 = (int32_t*)malloc(sizeof(int32_t) * data_len);
    memset(ave_result1,0,sizeof(ave_result1));
    memset(ave_result2,0,sizeof(ave_result2));

    sum_move(sample_data, data_len, 5, ave_result1, tail5); /* 2.56K */

    for (uint8_t i = 0; i < 5; i ++) {
        tail5[i] = sample_data[data_len-5+i];
    }

    sum_move(ave_result1, data_len, 1<<sample_order, ave_result2, tail256); /* 50HZ */
    for (uint16_t i = 0; i< (1<<sample_order); i++) {
        tail256[i] = ave_result1[data_len - (1<<sample_order) + i];
    }

    // discrete wavelet analysis
    dwt(ave_result2, wave_analysis, temp, h, g, data_len, wave_len);

    // store the detailed results, to be analyzed later
    //float result_data[VS_DATA_LEN / 2];
	float* result_data = (float*)malloc(sizeof(float) * (data_len / 2));
    for (uint16_t i = 0; i < data_len / 2; i++) {
        result_data[i] = wave_analysis[i];
    }

    // use pauta criterion to detect any abnormal data in the result datas
    mask = statis_decision(phase, result_data, data_len/2, sample_order, exp_pos, ave_mag, preamble_bits_data,sample_data);
	free(wave_analysis);
	free(temp);
	free(result_data);
    free(ave_result1);
    free(ave_result2);
    return mask;
}

