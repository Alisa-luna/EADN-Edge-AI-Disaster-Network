// ============================================
// dual_encoder_inference.c
// 双编码器C推理 — MCU部署版本
// ============================================

#include "dual_encoder_weights.h"
#include <math.h>
#include <string.h>

#define INPUT_LEN 30
#define TAU_CONV1_OUT 16
#define TAU_CONV2_OUT 32
#define TAU_CONV3_OUT 32
#define SP_CONV1_OUT 16
#define SP_CONV2_OUT 32
#define SP_CONV3_OUT 32

// 缓冲区
static float conv1_out[32 * 30];
static float conv2_out[32 * 30];
static float conv3_out[32 * 30];
static float pooled[32];
static float fc_hidden[32];

// ReLU
static inline float relu(float x) { return x > 0 ? x : 0; }

// Conv1d (padding=same)
static void conv1d_same(const float* input, int in_len,
                         const float* weight, const float* bias,
                         int in_ch, int out_ch, int kernel,
                         float* output) {
    int pad = kernel / 2;
    for (int oc = 0; oc < out_ch; oc++) {
        for (int t = 0; t < in_len; t++) {
            float sum = bias[oc];
            for (int ic = 0; ic < in_ch; ic++) {
                for (int k = 0; k < kernel; k++) {
                    int pos = t + k - pad;
                    if (pos >= 0 && pos < in_len) {
                        int w_idx = ((oc * in_ch) + ic) * kernel + k;
                        sum += input[ic * in_len + pos] * weight[w_idx];
                    }
                }
            }
            output[oc * in_len + t] = sum;
        }
    }
}

// AdaptiveAvgPool1d → 1
static void global_avg_pool(const float* input, int channels, int length, float* output) {
    for (int c = 0; c < channels; c++) {
        float sum = 0;
        for (int t = 0; t < length; t++) sum += input[c * length + t];
        output[c] = sum / length;
    }
}

// Linear
static void linear(const float* input, const float* weight, const float* bias,
                    int in_dim, int out_dim, int use_relu, float* output) {
    for (int o = 0; o < out_dim; o++) {
        float sum = bias[o];
        for (int i = 0; i < in_dim; i++) sum += input[i] * weight[o * in_dim + i];
        output[o] = use_relu ? relu(sum) : sum;
    }
}

// 完整编码器推理
static float encode_spectral(const float* input) {
    conv1d_same(input, INPUT_LEN, tau_conv1_weight, tau_conv1_bias, 1, 16, 3, conv1_out);
    for (int i = 0; i < 16*30; i++) conv1_out[i] = relu(conv1_out[i]);
    
    conv1d_same(conv1_out, INPUT_LEN, tau_conv2_weight, tau_conv2_bias, 16, 32, 3, conv2_out);
    for (int i = 0; i < 32*30; i++) conv2_out[i] = relu(conv2_out[i]);
    
    conv1d_same(conv2_out, INPUT_LEN, tau_conv3_weight, tau_conv3_bias, 32, 32, 3, conv3_out);
    for (int i = 0; i < 32*30; i++) conv3_out[i] = relu(conv3_out[i]);
    
    global_avg_pool(conv3_out, 32, INPUT_LEN, pooled);
    linear(pooled, tau_fc0_weight, tau_fc0_bias, 32, 32, 1, fc_hidden);
    linear(fc_hidden, tau_fc1_weight, tau_fc1_bias, 32, 1, 0, fc_hidden);
    
    return fc_hidden[0];  // log10(τc)
}

static float encode_temporal(const float* input) {
    conv1d_same(input, INPUT_LEN, sp_conv1_weight, sp_conv1_bias, 1, 16, 7, conv1_out);
    for (int i = 0; i < 16*30; i++) conv1_out[i] = relu(conv1_out[i]);
    
    conv1d_same(conv1_out, INPUT_LEN, sp_conv2_weight, sp_conv2_bias, 16, 32, 7, conv2_out);
    for (int i = 0; i < 32*30; i++) conv2_out[i] = relu(conv2_out[i]);
    
    conv1d_same(conv2_out, INPUT_LEN, sp_conv3_weight, sp_conv3_bias, 32, 32, 5, conv3_out);
    for (int i = 0; i < 32*30; i++) conv3_out[i] = relu(conv3_out[i]);
    
    global_avg_pool(conv3_out, 32, INPUT_LEN, pooled);
    linear(pooled, sp_fc0_weight, sp_fc0_bias, 32, 32, 1, fc_hidden);
    linear(fc_hidden, sp_fc1_weight, sp_fc1_bias, 32, 1, 0, fc_hidden);
    
    return fc_hidden[0];  // S-P时间(秒)
}

// ==================== 主推理函数 ====================

void dual_encoder_predict(const float* onset_normalized,  // 30个归一化采样点
                           float* tau_c_log10,              // 输出: log10(τc)
                           float* sp_time_sec) {            // 输出: S-P时间(秒)
    *tau_c_log10 = encode_spectral(onset_normalized);
    *sp_time_sec = encode_temporal(onset_normalized);
}

// ==================== 完整推理链 ====================

void full_inference_chain(const float* onset_raw,      // 30个原始采样点
                           float* s_peak_gal,           // 输出: S波峰值(gal)
                           float* tau_c_sec,            // 输出: τc(秒)
                           float* sp_time_sec,          // 输出: S-P时间(秒)
                           float* distance_km,          // 输出: 震中距(km)
                           float* warning_time_sec) {   // 输出: 预警时间(秒)
    
    // 1. 归一化初动
    float onset_max = 0;
    for (int i = 0; i < 30; i++) {
        float abs_val = fabs(onset_raw[i]);
        if (abs_val > onset_max) onset_max = abs_val;
    }
    onset_max += 1e-10f;
    
    float onset_norm[30];
    for (int i = 0; i < 30; i++) onset_norm[i] = onset_raw[i] / onset_max;
    
    // 2. 物理公式：S波峰值
    float full_pd_est = onset_max * 2.5f;
    float s_peak_log = 0.5f + 0.9f * log10f(full_pd_est + 1e-10f);
    *s_peak_gal = powf(10.0f, s_peak_log);
    
    // 3. 神经网络：τc + S-P时间
    float tau_log, sp_time;
    dual_encoder_predict(onset_norm, &tau_log, &sp_time);
    
    *tau_c_sec = powf(10.0f, tau_log);
    *sp_time_sec = sp_time;
    
    // 4. 震中距
    *distance_km = 8.0f * sp_time;
    
    // 5. 预警时间 = S-P时间
    *warning_time_sec = sp_time;
}