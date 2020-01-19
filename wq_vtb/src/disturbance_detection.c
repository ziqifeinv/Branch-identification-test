#include "disturbance_detection.h"
#include <stdlib.h>
#include "wq_debug.h"
#include "wq_version.h"

void get_pos_max_magnitude(float* data, uint16_t* group_pos, uint8_t num, float* max_val) {
    for (uint8_t i = 0; i < num; i++) {
        if (abs(data[group_pos[i]]) > * max_val) {
            *max_val = abs(data[group_pos[i]]);
        }
    }
}

float get_min(float* val, uint8_t len, uint8_t* idx) {
    float min_val = val[0];
    for (uint16_t i = 0; i < len; i++) {
        if (val[i] < min_val) {
            *idx = i;
            min_val = val[i];
        }
    }
    return min_val;
}

//sort the position, and adjust its magnitude accordingly
void sort(uint16_t* arr, float* val, uint8_t size) {
    uint8_t i, j;
    uint16_t tmp;
    float v;
    for (i = 0; i < size - 1; i++) {
        for (j = 0; j < size - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                tmp = arr[j]; v = val[j];
                arr[j] = arr[j + 1]; val[j] = val[j + 1];
                arr[j + 1] = tmp; val[j + 1] = v;
            }
        }
    }
}

uint8_t check_sign(float* data, uint16_t* current_group_pos, uint8_t num, uint16_t ref_pos) {
    uint8_t cnt = 0, tot_cnt = 0;
    for (uint8_t i = 0; i < num; i++) {
        if (current_group_pos[i] - ref_pos <= 10) {
            tot_cnt++;
            if (data[current_group_pos[i]] > 0) {
                cnt++;
            }
        }
    }
    wq_dbg_printf("cnt %d, tol %d.", cnt, tot_cnt);
    if ((cnt >= 5) || (cnt == tot_cnt)) {
        return 1;
    }
    return 0;
}

uint8_t check_sign_new(float* data, uint16_t* current_group_pos, uint8_t num, uint16_t ref_pos) {
    //uint16_t up_pos[num], low_pos[num], first_half[num];
    uint16_t* up_pos = (uint16_t*)malloc(sizeof(uint16_t) * num);
    uint16_t* low_pos = (uint16_t*)malloc(sizeof(uint16_t) * num);
    uint16_t* first_half = (uint16_t*)malloc(sizeof(uint16_t) * num);
    memset(up_pos, 0, sizeof(uint16_t) * num);
    memset(low_pos, 0, sizeof(uint16_t) * num);
    memset(first_half, 0, sizeof(uint16_t) * num);
    uint8_t up_num = 0, low_num = 0, i, tot_cnt = 0;

    for (i = 0; i < num; i++) {
        if (current_group_pos[i] - ref_pos < 64) {
            first_half[tot_cnt] = current_group_pos[i];
            tot_cnt++;
        }
    }

    float max_mag = 0, min_mag = 0;

    for (i = 0; i < tot_cnt; i++) {
        if (data[first_half[i]] > 0) {
            up_num++;
        }
        else {
            low_num++;
        }
        if (data[first_half[i]] > max_mag) {
            max_mag = data[first_half[i]];
        }
        if (data[first_half[i]] < min_mag) {
            min_mag = data[first_half[i]];
        }
    }

    wq_dbg_printf("positive number %d, negative number %d.", up_num, low_num);
    if ((up_num > low_num) || (max_mag > abs(min_mag))) {
        return 1;
    }

    free(up_pos);
    free(low_pos);
    free(first_half);
    return 0;
}

uint8_t check_sign_new1(float* data, uint16_t data_len, uint16_t ref_pos) {
    uint16_t start_pos, end_pos;
    start_pos = (ref_pos < 138) ? 0 : (ref_pos - 138);
    end_pos = (ref_pos > (data_len - 139)) ? (data_len - 1) : (ref_pos + 138);

    float temp_max = 0, temp_min = 0;
    uint16_t max_pos = 0, min_pos = 0;
    for (uint16_t i = start_pos; i < end_pos + 1; i++) {
        if (data[i] > temp_max) {
            temp_max = data[i];
            max_pos = i;
        }
        if (data[i] < temp_min) {
            temp_min = data[i];
            min_pos = i;
        }
    }
    if (min_pos < max_pos) {
        return 1;
    }
    else {
        return 0;
    }
}

uint8_t check_sign_new2(float* data, uint16_t data_len, uint16_t ref_pos)
{
    uint16_t start_pos = 0, end_pos = 0;
    float result = 0.0;

    start_pos = (ref_pos < 138) ? 0 : (ref_pos - 138);
    end_pos = (ref_pos > (data_len - 139)) ? (data_len - 1) : (ref_pos + 138);

    float temp_max = 0, temp_min = 0;
    uint16_t max_pos = 0, min_pos = 0;
    for (uint16_t i = start_pos; i < end_pos + 1; i++) {
        if (data[i] > temp_max) {
            temp_max = data[i];
            max_pos = i;
        }
        if (data[i] < temp_min) {
            temp_min = data[i];
            min_pos = i;
        }
    }
    uint8_t win = 20, compFactor = 5;
    uint16_t left_min_pos = 0, left_max_pos = 0;
    float left_min = 0, left_max = 0;
    if (min_pos < max_pos) {
        for (uint16_t j = max(0, min_pos - win); j < min(min_pos + win, data_len); j++) {
            if (data[j] >= left_max) {
                left_max = data[j];
                left_max_pos = j;
            }
        }
        if ((min_pos < left_max_pos) || (-temp_min >= compFactor * left_max)) {
            result = 1;
            goto out;
        }
        else {
            result = -temp_min / (-temp_min + left_max);
            goto out;
        }
    }
    else {
        for (uint16_t k = max(0, max_pos - win); k < min(max_pos + win, data_len); k++){
            if (data[k] <= left_min) {
                left_min = data[k];
                left_min_pos = k;
            }
        }
        if ((max_pos > left_min_pos) || (temp_max >= compFactor * (-left_min))) {
            result = 0;
            goto out;
        }
        else {
            result = -left_min / (temp_max - left_min);
            goto out;
        }
    }
out:
    return (uint8_t)(result * 100);
}

void convolution(int32_t* data, float* core, float* conv_results, uint16_t n)
{
    uint16_t i = 0;
    //int32_t data_ext[n + 2];
    int32_t* data_ext = malloc(sizeof(int32_t) * (n + 2));
    int32_t tmp1, tmp2;

    for (i = 0; i < n + 2; i++) {
        if (i == 0) {
            data_ext[i] = data[0];
            continue;
        }
        if (i == n + 1) {
            data_ext[i] = data[i - 2];
            continue;
        }
        data_ext[i] = data[i - 1];
    }

    for (i = 0; i < n + 1; i++) {
        conv_results[i] = 0;
    }

    for (i = 0; i < n + 1; i++) {
        tmp1 = data_ext[i], tmp2 = data_ext[i + 1];
        conv_results[i] = tmp1 * core[1] + tmp2 * core[0];
    }
    free(data_ext);
}

void dwt(int32_t* input, float* output, float* temp, float* g, uint16_t n)
{
    convolution(input, g, temp, n);

    for (uint16_t i = 1; i < n + 1; i += 2) {
        output[(i - 1) / 2] = temp[i];//wave factor
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

uint8_t statis_decision(uint32_t phase, float* data, uint16_t data_length, uint8_t sample_order,
    int32_t expect_pos, float ave_mag, wq_prm_bit_val_t* preamble_bits_data)
{
    float sum = 0.0;
    float ave = 0.0;
    float temp_dis = 0.0;
    uint16_t len = 1 << (sample_order - 1);
    uint16_t i;

    //calculate mean of data
    for (i = 0; i < data_length; i++) {
        sum += data[i];
    }
    ave = sum / (data_length);
    // calculate variance of data
    for (i = 0; i < data_length; i++) {
        float dis = ave - data[i];
        temp_dis += dis * dis;
    }
    float var = math_sqrt_newton_iterative(temp_dis / (data_length - 1));
    uint16_t temp_num = 0, temp_pos[512];

    float coeff = 2.0;

    for (i = 0; i < data_length; i++) {
        if ((data[i] > ave + coeff * var) || (data[i] < ave - coeff * var)) {
            temp_pos[temp_num] = i;//positions have been sorted
            temp_num++; /* TODO : overflow */
        }
    }

    //sort(temp_pos, temp_num);

    wq_dbg_printf("possible found samples is %d\n", temp_num);

    //initialized the output struct
    preamble_bits_data->bit_cnt = 0;
    memset(&preamble_bits_data->bit_pos[0], 0, sizeof(uint16_t) * PREAMBLE_BIT_GROUP);
    memset(&preamble_bits_data->mag_sign[0], 0, sizeof(uint8_t) * PREAMBLE_BIT_GROUP);
    memset(&preamble_bits_data->mag_val[0], 0, sizeof(float) * PREAMBLE_BIT_GROUP);

    if (expect_pos < 0) { //if there is no expect position, certain possible positions need to return
        if (temp_num < 3) { //if not find any possible position, return directly
            preamble_bits_data->bit_cnt = 0;
            return 0;
        }

        //reference position, initialized with first position
        uint16_t ref_pos = temp_pos[0];
        uint16_t current_group_pos[138], group_pos_left[25];
        uint8_t sign_group_pos[25];
        float max_mag[25];
        //initialized the new variables
        memset(current_group_pos, 0, sizeof(current_group_pos));
        memset(group_pos_left, 0, sizeof(group_pos_left));
        memset(max_mag, 0, sizeof(max_mag));
        memset(sign_group_pos, 0, sizeof(sign_group_pos));
        uint8_t group_num = 1, count = 0;

        for (i = 0; i < temp_num; i++) {
            //if the select position is near by reference position, put them into one group
            if (temp_pos[i] - ref_pos < len + 10) {
                current_group_pos[count] = temp_pos[i];
                count++;//number of position in current group
            }
            else {
                //deal with current group: get the max magnitude and its position
                get_pos_max_magnitude(data, current_group_pos, count, &max_mag[group_num - 1]);
                //save the reference position
                group_pos_left[group_num - 1] = ref_pos;
                //check the sign of current group
                //sign_group_pos[group_num-1] = check_sign_new(data, current_group_pos, count, ref_pos);
#if(TOPO_VERSION >= TOPO_V3_4)
                sign_group_pos[group_num - 1] = check_sign_new2(data, data_length, ref_pos);
#else if(TOPO_VERSION >= TOPO_V3_3)
                sign_group_pos[group_num - 1] = check_sign_new1(data, data_length, ref_pos);
#endif
                //reset current position group
                memset(current_group_pos, 0, sizeof(current_group_pos));
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

        wq_dbg_printf("find group number is %d\n", group_num);

        //deal with the last group: get the max magnitude and its position
        get_pos_max_magnitude(data, current_group_pos, count, &max_mag[group_num - 1]);
        group_pos_left[group_num - 1] = ref_pos;
        //sign_group_pos[group_num-1] = check_sign_new(data, current_group_pos, count, ref_pos);
#if(TOPO_VERSION >= TOPO_V3_4)
        sign_group_pos[group_num - 1] = check_sign_new2(data, data_length, ref_pos);
#else if(TOPO_VERSION >= TOPO_V3_3)
        sign_group_pos[group_num - 1] = check_sign_new1(data, data_length, ref_pos);
#endif
        //if the found number of groups is less than expectation, return directly
        if (group_num <= PREAMBLE_BIT_GROUP) {
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
        for (i = 0; i < group_num; i++) {
            float min = get_min(value_pos, PREAMBLE_BIT_GROUP, &idx);
            if (max_mag[i] > min) {
                pos[idx] = group_pos_left[i];
                value_pos[idx] = max_mag[i];
            }
        }
        //sort the select group position
        sort(pos, value_pos, PREAMBLE_BIT_GROUP);
        //search the sign of the selected group
        for (i = 0; i < PREAMBLE_BIT_GROUP; i++) {
            for (uint8_t j = 0; j < group_num; j++) {
                if (pos[i] == group_pos_left[j]) {
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
    else {//if there exist an expect position, return demod bit according to the results around the expect position

        //need to delete later!!!!!!!!!!!!!!!!!!!!!!!
        //reference position, initialized with first position
        uint16_t ref_pos = temp_pos[0];
        uint16_t current_group_pos[138], group_pos_left[25];
        uint8_t sign_group_pos[25];
        float max_mag[25];
        memset(current_group_pos, 0, sizeof(current_group_pos));
        memset(group_pos_left, 0, sizeof(group_pos_left));
        memset(max_mag, 0, sizeof(max_mag));
        memset(sign_group_pos, 0, sizeof(sign_group_pos));
        uint8_t group_num = 1, count = 0, ret = 0;

        for (i = 0; i < temp_num; i++) {
            //if the select position is near by reference position, put them into one group
            if (temp_pos[i] - ref_pos < len + 10) {
                current_group_pos[count] = temp_pos[i];
                count++;//number of position in current group
            }
            else {
                //deal with current group: get the max magnitude and its position
                get_pos_max_magnitude(data, current_group_pos, count, &max_mag[group_num - 1]);
                group_pos_left[group_num - 1] = ref_pos;
#if(TOPO_VERSION >= TOPO_V3_4)
                sign_group_pos[group_num - 1] = check_sign_new2(data, data_length, ref_pos);
#else if(TOPO_VERSION >= TOPO_V3_3)
                sign_group_pos[group_num - 1] = check_sign_new1(data, data_length, ref_pos);
#endif
                //reset current position group
                memset(current_group_pos, 0, sizeof(current_group_pos));
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

        wq_dbg_printf("find group number is %d\n", group_num);

        //deal with the last group: get the max magnitude and its position
        get_pos_max_magnitude(data, current_group_pos, count, &max_mag[group_num - 1]);
        group_pos_left[group_num - 1] = ref_pos;
        //sign_group_pos[group_num-1] = check_sign_new(data, current_group_pos, count, ref_pos);
#if(TOPO_VERSION >= TOPO_V3_4)
        sign_group_pos[group_num - 1] = check_sign_new2(data, data_length, ref_pos);
#else if(TOPO_VERSION >= TOPO_V3_3)
        sign_group_pos[group_num - 1] = check_sign_new1(data, data_length, ref_pos);
#endif
        //if the found number of groups is less than expectation, return directly
        if (group_num <= PREAMBLE_BIT_GROUP) {
            preamble_bits_data->bit_cnt = group_num;
            memcpy(&preamble_bits_data->bit_pos[0], group_pos_left, sizeof(uint16_t) * group_num);
            memcpy(&preamble_bits_data->mag_sign[0], sign_group_pos, sizeof(uint8_t) * group_num);
            memcpy(&preamble_bits_data->mag_val[0], max_mag, sizeof(float) * group_num);
            uint16_t find_sample = 0; float max_mag_expect_pos = 0.0;
            for (i = 0; i < temp_num; i++) {
                if (abs(temp_pos[i] - expect_pos) < (len)) {
                    find_sample++;
                    if (abs(data[temp_pos[i]]) > max_mag_expect_pos) {
                        max_mag_expect_pos = abs(data[temp_pos[i]]);
                    }
                }
            }

            if ((find_sample >= 5) && (3 * max_mag_expect_pos >= ave_mag)) {
                ret = 1;
            }
            wq_dbg_printf("case1 wave len %d, average magnitude %d, current group max magnitude %d.",
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

        for (i = 0; i < group_num; i++) {
            float min = get_min(value_pos, PREAMBLE_BIT_GROUP, &idx);
            if (max_mag[i] > min) {
                pos[idx] = group_pos_left[i];
                value_pos[idx] = max_mag[i];
            }
        }

        sort(pos, value_pos, PREAMBLE_BIT_GROUP);
        for (i = 0; i < PREAMBLE_BIT_GROUP; i++) {
            for (uint8_t j = 0; j < group_num; j++) {
                if (pos[i] == group_pos_left[j]) {
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
        for (i = 0; i < temp_num; i++) {
            if (abs(temp_pos[i] - expect_pos) < (len)) {
                find_sample++;
                if (abs(data[temp_pos[i]]) > max_mag_expect_pos) {
                    max_mag_expect_pos = abs(data[temp_pos[i]]);
                }
            }
        }
        if ((find_sample >= 5) && (3 * max_mag_expect_pos >= ave_mag)) {
            ret = 1;
        }
        wq_dbg_printf("case2 wave len %d, average magnitude %d, current group max magnitude %d.",
            find_sample, (int)ave_mag, (int)max_mag_expect_pos);
        return ret;
    }
}

void sum_move(int32_t* sample_data, uint16_t data_len,
    uint16_t moveLen, int32_t* res)
{
    //int32_t buff[moveLen];
    int32_t* buff = malloc(sizeof(int32_t) * moveLen);
    uint16_t idx = 0;
    int32_t sum = 0;

    for (uint16_t i = 0; i < moveLen; i++) {
        buff[i] = sample_data[i];
        sum += buff[i];
    }

    for (uint16_t j = moveLen; j < data_len; j++) {
        idx = j % (moveLen);
        sum += sample_data[j] - buff[idx];
        buff[idx] = sample_data[j];
        res[j - moveLen] = sum;
    }
    free(buff);
}

uint8_t disturb_detection(uint32_t phase, int32_t* sample_data, uint16_t data_len_all,
    uint8_t sample_order, int32_t exp_pos, float ave_mag, wq_prm_bit_val_t* preamble_bits_data)
{
    uint16_t data_len = data_len_all - 5 - 256;
    //float wave_analysis[(data_len + 1) / 2];
    //float temp[data_len + 1];
    //int32_t ave_result1[data_len + 256];
    //int32_t ave_result2[data_len];
    float* wave_analysis = malloc(sizeof(float) * ((data_len + 1) / 2));
    float* temp = malloc(sizeof(float) * (data_len + 1));
    int32_t* ave_result1 = malloc(sizeof(int32_t) * (data_len + 256));
    int32_t* ave_result2 = malloc(sizeof(int32_t) * (data_len));

    static float g[] = { -0.7071, 0.7071 };
    uint8_t interval = 10;
    uint8_t mask = 0;

    for (uint16_t i = 0; i < data_len_all; i++) {
        if (abs(sample_data[i]) > 50000) {
            if (i < interval) {
                continue;
            }
            if (i < ((1 << sample_order) + (interval << 1) + 1)) {
                for (uint8_t j = 0; j <= interval; j++) {
                    sample_data[i - (interval << 1) + j] =
                        sample_data[i + (1 << sample_order) - (interval << 1) + j];
                }
            }
            else {
                for (uint8_t j = 0; j <= interval; j++) {
                    sample_data[i - (interval << 1) + j] =
                        sample_data[i - (1 << sample_order) - (interval << 1) + j];
                }
            }
        }
    }

    memset(ave_result1, 0, sizeof(*ave_result1));
    memset(ave_result2, 0, sizeof(*ave_result2));

    sum_move(sample_data, data_len_all, 5, ave_result1); /* 2.56K */

    sum_move(ave_result1, data_len + 256, 256, ave_result2); /* 50HZ */

    // discrete wavelet analysis
    dwt(ave_result2, wave_analysis, temp, g, data_len);

    // use pauta criterion to detect any abnormal data in the result datas
    mask = statis_decision(phase, wave_analysis, (data_len + 1) / 2, sample_order, exp_pos, ave_mag, preamble_bits_data);
    free(wave_analysis);
    free(temp);
    free(ave_result1);
    free(ave_result2);
    return mask;
}

