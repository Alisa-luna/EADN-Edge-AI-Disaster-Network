#include "ae_model.h"
#include "ae_weights.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>

// ==================== 推理用 PSRAM 缓冲区指针 ====================
static float* enc_c1 = NULL;
static float* enc_c2 = NULL;
static float* enc_c3 = NULL;
static float* pooled = NULL;
static float* latent_buf = NULL;
static float* dec_fc = NULL;
static float* dec_c1_buf = NULL;
static float* dec_c2_buf = NULL;
static float* dec_c3_buf = NULL;
static float* dec_out = NULL;

static AEBaseline global_baseline;

// ==================== 可写权重 PSRAM ====================
static float* w_dec_ct3_w = NULL;
static float* w_dec_ct3_b = NULL;
static float* w_dec_ct2_w = NULL;
static float* w_dec_ct2_b = NULL;
static float* w_dec_ct1_w = NULL;
static float* w_dec_ct1_b = NULL;
static float* w_fc_dec_w = NULL;
static float* w_fc_dec_b = NULL;

// ==================== 原始权重备份 PSRAM ====================
static float* orig_dec_ct3_w = NULL;
static float* orig_dec_ct3_b = NULL;
static float* orig_dec_ct2_w = NULL;
static float* orig_dec_ct2_b = NULL;
static float* orig_dec_ct1_w = NULL;
static float* orig_dec_ct1_b = NULL;
static float* orig_fc_dec_w = NULL;
static float* orig_fc_dec_b = NULL;
static bool weights_saved = false;

// ==================== 训练用 PSRAM 缓冲区指针 ====================
static float* enc_c1_t = NULL;
static float* enc_c2_t = NULL;
static float* enc_c3_t = NULL;
static float* pooled_t = NULL;
static float* latent_t = NULL;
static float* dec_fc_t = NULL;
static float* dec_c1_t = NULL;
static float* dec_c2_t = NULL;
static float* dec_c3_t = NULL;
static float* output_t = NULL;
static float* d_out = NULL;
static float* d_c3 = NULL;
static float* d_c2 = NULL;
static float* d_c1 = NULL;
static float* d_fc = NULL;
static bool training_buffers_allocated = false;
static bool all_buffers_allocated = false;

// ==================== 辅助函数 ====================
static inline float relu(float x) { return x > 0 ? x : 0; }
static void sort_floats(float* arr, int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (arr[i] > arr[j]) {
                float tmp = arr[i];
                arr[i] = arr[j];
                arr[j] = tmp;
            }
        }
    }
}
static void conv1d_same(const float* input, int in_ch, int in_len,
                         const float* weight, const float* bias,
                         int out_ch, int kernel, int stride,
                         float* output) {
    int out_len = in_len / stride;
    int pad = kernel / 2;
    for (int oc = 0; oc < out_ch; oc++) {
        for (int t = 0; t < out_len; t++) {
            float sum = bias[oc];
            for (int ic = 0; ic < in_ch; ic++) {
                for (int k = 0; k < kernel; k++) {
                    int pos = t * stride + k - pad;
                    if (pos >= 0 && pos < in_len) {
                        sum += input[ic * in_len + pos] * weight[((oc * in_ch) + ic) * kernel + k];
                    }
                }
            }
            output[oc * out_len + t] = sum;
        }
    }
}

static void convt1d(const float* input, int in_ch, int in_len,
                     const float* weight, const float* bias,
                     int out_ch, int kernel, int stride, int out_pad,
                     float* output) {
    int out_len = in_len * stride + out_pad;
    int pad = kernel / 2;
    memset(output, 0, out_ch * out_len * sizeof(float));
    for (int ic = 0; ic < in_ch; ic++) {
        for (int oc = 0; oc < out_ch; oc++) {
            for (int t = 0; t < in_len; t++) {
                float val = input[ic * in_len + t];
                for (int k = 0; k < kernel; k++) {
                    int pos = t * stride + k - pad;
                    if (pos >= 0 && pos < out_len) {
                        output[oc * out_len + pos] += val * weight[(ic * out_ch + oc) * kernel + k];
                    }
                }
            }
        }
    }
    for (int oc = 0; oc < out_ch; oc++)
        for (int t = 0; t < out_len; t++)
            output[oc * out_len + t] += bias[oc];
}

static void batchnorm1d(float* data, int ch, int len,
                         const float* rm, const float* rv,
                         const float* gamma, const float* beta) {
    float eps = 1e-5f;
    for (int c = 0; c < ch; c++) {
        float inv_std = 1.0f / sqrtf(rv[c] + eps);
        for (int t = 0; t < len; t++) {
            float x = data[c * len + t];
            data[c * len + t] = (x - rm[c]) * inv_std * gamma[c] + beta[c];
        }
    }
}

static void linear(const float* input, const float* weight, const float* bias,
                   int in_dim, int out_dim, int use_relu, float* output) {
    for (int o = 0; o < out_dim; o++) {
        float s = bias[o];
        for (int i = 0; i < in_dim; i++) s += input[i] * weight[o * in_dim + i];
        output[o] = use_relu ? relu(s) : s;
    }
}

static void global_avg_pool(const float* input, int ch, int len, float* output) {
    for (int c = 0; c < ch; c++) {
        float sum = 0;
        for (int t = 0; t < len; t++) sum += input[c * len + t];
        output[c] = sum / len;
    }
}

// ==================== 统一分配 ====================
static bool allocate_all_buffers() {
    if (all_buffers_allocated) return true;

    enc_c1 = (float*)heap_caps_malloc(16*100*sizeof(float), MALLOC_CAP_SPIRAM);
    enc_c2 = (float*)heap_caps_malloc(32*50*sizeof(float), MALLOC_CAP_SPIRAM);
    enc_c3 = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);
    pooled = (float*)heap_caps_malloc(64*sizeof(float), MALLOC_CAP_SPIRAM);
    latent_buf = (float*)heap_caps_malloc(16*sizeof(float), MALLOC_CAP_SPIRAM);
    dec_fc = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);
    dec_c1_buf = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);
    dec_c2_buf = (float*)heap_caps_malloc(32*50*sizeof(float), MALLOC_CAP_SPIRAM);
    dec_c3_buf = (float*)heap_caps_malloc(16*100*sizeof(float), MALLOC_CAP_SPIRAM);
    dec_out = (float*)heap_caps_malloc(6*200*sizeof(float), MALLOC_CAP_SPIRAM);

    w_dec_ct3_w = (float*)heap_caps_malloc(16*6*7*sizeof(float), MALLOC_CAP_SPIRAM);
    w_dec_ct3_b = (float*)heap_caps_malloc(6*sizeof(float), MALLOC_CAP_SPIRAM);
    w_dec_ct2_w = (float*)heap_caps_malloc(32*16*5*sizeof(float), MALLOC_CAP_SPIRAM);
    w_dec_ct2_b = (float*)heap_caps_malloc(16*sizeof(float), MALLOC_CAP_SPIRAM);
    w_dec_ct1_w = (float*)heap_caps_malloc(64*32*5*sizeof(float), MALLOC_CAP_SPIRAM);
    w_dec_ct1_b = (float*)heap_caps_malloc(32*sizeof(float), MALLOC_CAP_SPIRAM);
    w_fc_dec_w = (float*)heap_caps_malloc(64*25*16*sizeof(float), MALLOC_CAP_SPIRAM);
    w_fc_dec_b = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);

    orig_dec_ct3_w = (float*)heap_caps_malloc(16*6*7*sizeof(float), MALLOC_CAP_SPIRAM);
    orig_dec_ct3_b = (float*)heap_caps_malloc(6*sizeof(float), MALLOC_CAP_SPIRAM);
    orig_dec_ct2_w = (float*)heap_caps_malloc(32*16*5*sizeof(float), MALLOC_CAP_SPIRAM);
    orig_dec_ct2_b = (float*)heap_caps_malloc(16*sizeof(float), MALLOC_CAP_SPIRAM);
    orig_dec_ct1_w = (float*)heap_caps_malloc(64*32*5*sizeof(float), MALLOC_CAP_SPIRAM);
    orig_dec_ct1_b = (float*)heap_caps_malloc(32*sizeof(float), MALLOC_CAP_SPIRAM);
    orig_fc_dec_w = (float*)heap_caps_malloc(64*25*16*sizeof(float), MALLOC_CAP_SPIRAM);
    orig_fc_dec_b = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);

    if (enc_c1 && enc_c2 && enc_c3 && pooled && latent_buf && dec_fc &&
        dec_c1_buf && dec_c2_buf && dec_c3_buf && dec_out &&
        w_dec_ct3_w && w_dec_ct3_b && w_dec_ct2_w && w_dec_ct2_b &&
        w_dec_ct1_w && w_dec_ct1_b && w_fc_dec_w && w_fc_dec_b &&
        orig_dec_ct3_w && orig_dec_ct3_b && orig_dec_ct2_w && orig_dec_ct2_b &&
        orig_dec_ct1_w && orig_dec_ct1_b && orig_fc_dec_w && orig_fc_dec_b) {
        all_buffers_allocated = true;
        Serial.println("[AE] All buffers allocated in PSRAM");
        return true;
    }
    Serial.println("[AE] PSRAM allocation FAILED!");
    return false;
}

static bool allocate_training_buffers() {
    if (training_buffers_allocated) return true;

    enc_c1_t = (float*)heap_caps_malloc(16*100*sizeof(float), MALLOC_CAP_SPIRAM);
    enc_c2_t = (float*)heap_caps_malloc(32*50*sizeof(float), MALLOC_CAP_SPIRAM);
    enc_c3_t = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);
    pooled_t = (float*)heap_caps_malloc(64*sizeof(float), MALLOC_CAP_SPIRAM);
    latent_t = (float*)heap_caps_malloc(16*sizeof(float), MALLOC_CAP_SPIRAM);
    dec_fc_t = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);
    dec_c1_t = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);
    dec_c2_t = (float*)heap_caps_malloc(32*50*sizeof(float), MALLOC_CAP_SPIRAM);
    dec_c3_t = (float*)heap_caps_malloc(16*100*sizeof(float), MALLOC_CAP_SPIRAM);
    output_t = (float*)heap_caps_malloc(6*200*sizeof(float), MALLOC_CAP_SPIRAM);
    d_out = (float*)heap_caps_malloc(6*200*sizeof(float), MALLOC_CAP_SPIRAM);
    d_c3  = (float*)heap_caps_malloc(16*100*sizeof(float), MALLOC_CAP_SPIRAM);
    d_c2  = (float*)heap_caps_malloc(32*50*sizeof(float), MALLOC_CAP_SPIRAM);
    d_c1  = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);
    d_fc  = (float*)heap_caps_malloc(64*25*sizeof(float), MALLOC_CAP_SPIRAM);

    if (enc_c1_t && enc_c2_t && enc_c3_t && pooled_t && latent_t &&
        dec_fc_t && dec_c1_t && dec_c2_t && dec_c3_t && output_t &&
        d_out && d_c3 && d_c2 && d_c1 && d_fc) {
        training_buffers_allocated = true;
        return true;
    }
    return false;
}

// ==================== 权重管理 ====================
static void save_original_weights() {
    memcpy(orig_dec_ct3_w, ae_dec_ct3_w, 16*6*7*sizeof(float));
    memcpy(orig_dec_ct3_b, ae_dec_ct3_b, 6*sizeof(float));
    memcpy(orig_dec_ct2_w, ae_dec_ct2_w, 32*16*5*sizeof(float));
    memcpy(orig_dec_ct2_b, ae_dec_ct2_b, 16*sizeof(float));
    memcpy(orig_dec_ct1_w, ae_dec_ct1_w, 64*32*5*sizeof(float));
    memcpy(orig_dec_ct1_b, ae_dec_ct1_b, 32*sizeof(float));
    memcpy(orig_fc_dec_w, ae_fc_dec_w, 64*25*16*sizeof(float));
    memcpy(orig_fc_dec_b, ae_fc_dec_b, 64*25*sizeof(float));
    weights_saved = true;
    ae_reset_weights_to_original();
}

void ae_reset_weights_to_original() {
    if (!weights_saved) return;
    memcpy(w_dec_ct3_w, orig_dec_ct3_w, 16*6*7*sizeof(float));
    memcpy(w_dec_ct3_b, orig_dec_ct3_b, 6*sizeof(float));
    memcpy(w_dec_ct2_w, orig_dec_ct2_w, 32*16*5*sizeof(float));
    memcpy(w_dec_ct2_b, orig_dec_ct2_b, 16*sizeof(float));
    memcpy(w_dec_ct1_w, orig_dec_ct1_w, 64*32*5*sizeof(float));
    memcpy(w_dec_ct1_b, orig_dec_ct1_b, 32*sizeof(float));
    memcpy(w_fc_dec_w, orig_fc_dec_w, 64*25*16*sizeof(float));
    memcpy(w_fc_dec_b, orig_fc_dec_b, 64*25*sizeof(float));
}

// ==================== AE 初始化 ====================
void ae_init_weights() {
    memset(&global_baseline, 0, sizeof(AEBaseline));
    if (!allocate_all_buffers()) {
        Serial.println("[AE] FATAL: PSRAM allocation failed!");
    }
    save_original_weights();
}

// ==================== AE 推理 ====================
AEOutput ae_predict(const float* input, float* output, float* latent_out) {
    AEOutput ret = {0};
    if (!weights_saved) save_original_weights();

    // ---- Encoder ----
    conv1d_same(input, 6, 200, ae_enc_conv1_w, ae_enc_conv1_b, 16, 7, 2, enc_c1);
    batchnorm1d(enc_c1, 16, 100, ae_enc_bn1_rm, ae_enc_bn1_rv, ae_enc_bn1_w, ae_enc_bn1_b);
    for (int i = 0; i < 16*100; i++) enc_c1[i] = relu(enc_c1[i]);

    conv1d_same(enc_c1, 16, 100, ae_enc_conv2_w, ae_enc_conv2_b, 32, 5, 2, enc_c2);
    batchnorm1d(enc_c2, 32, 50, ae_enc_bn2_rm, ae_enc_bn2_rv, ae_enc_bn2_w, ae_enc_bn2_b);
    for (int i = 0; i < 32*50; i++) enc_c2[i] = relu(enc_c2[i]);

    conv1d_same(enc_c2, 32, 50, ae_enc_conv3_w, ae_enc_conv3_b, 64, 5, 2, enc_c3);
    for (int i = 0; i < 64*25; i++) enc_c3[i] = relu(enc_c3[i]);

    global_avg_pool(enc_c3, 64, 25, pooled);
    linear(pooled, ae_fc_enc_w, ae_fc_enc_b, 64, 16, false, latent_buf);

    // ---- Decoder（使用可写权重）----
    linear(latent_buf, w_fc_dec_w, w_fc_dec_b, 16, 64*25, true, dec_fc);
    memcpy(dec_c1_buf, dec_fc, 64*25*sizeof(float));

    convt1d(dec_c1_buf, 64, 25, w_dec_ct1_w, w_dec_ct1_b, 32, 5, 2, 1, dec_c2_buf);
    for (int i = 0; i < 32*50; i++) dec_c2_buf[i] = relu(dec_c2_buf[i]);

    convt1d(dec_c2_buf, 32, 50, w_dec_ct2_w, w_dec_ct2_b, 16, 5, 2, 1, dec_c3_buf);
    for (int i = 0; i < 16*100; i++) dec_c3_buf[i] = relu(dec_c3_buf[i]);

    convt1d(dec_c3_buf, 16, 100, w_dec_ct3_w, w_dec_ct3_b, 6, 7, 2, 1, dec_out);

    // ---- 计算误差 ----
    float mse = 0;
    for (int i = 0; i < 6*200; i++) {
        float d = input[i] - dec_out[i];
        mse += d * d;
    }
    mse /= (6*200);

    float lat_dist = 0;
    for (int i = 0; i < 16; i++) lat_dist += latent_buf[i] * latent_buf[i];
    lat_dist = sqrtf(lat_dist);

    float spec_err = 0;
    for (int c = 0; c < 6; c++) {
        float in_amp = 0, out_amp = 0;
        for (int t = 0; t < 100; t++) {
            in_amp += fabsf(input[c*200+t]);
            out_amp += fabsf(dec_out[c*200+t]);
        }
        spec_err += fabsf(in_amp - out_amp) / 100.0f;
    }
    spec_err /= 6;

    ret.mse = mse;
    ret.latent_dist = lat_dist;
    ret.spectral_err = spec_err;
    ret.score = 0;

    if (output) memcpy(output, dec_out, 6*200*sizeof(float));
    if (latent_out) memcpy(latent_out, latent_buf, 16*sizeof(float));

    return ret;
}

// ==================== 前向传播（带缓存）====================
static void ae_forward_with_cache(
    const float* input,
    float* out_enc_c1, float* out_enc_c2, float* out_enc_c3,
    float* out_pooled, float* out_latent,
    float* out_dec_fc, float* out_dec_c1, float* out_dec_c2, float* out_dec_c3,
    float* output
) {
    conv1d_same(input, 6, 200, ae_enc_conv1_w, ae_enc_conv1_b, 16, 7, 2, out_enc_c1);
    batchnorm1d(out_enc_c1, 16, 100, ae_enc_bn1_rm, ae_enc_bn1_rv, ae_enc_bn1_w, ae_enc_bn1_b);
    for (int i = 0; i < 16*100; i++) out_enc_c1[i] = relu(out_enc_c1[i]);

    conv1d_same(out_enc_c1, 16, 100, ae_enc_conv2_w, ae_enc_conv2_b, 32, 5, 2, out_enc_c2);
    batchnorm1d(out_enc_c2, 32, 50, ae_enc_bn2_rm, ae_enc_bn2_rv, ae_enc_bn2_w, ae_enc_bn2_b);
    for (int i = 0; i < 32*50; i++) out_enc_c2[i] = relu(out_enc_c2[i]);

    conv1d_same(out_enc_c2, 32, 50, ae_enc_conv3_w, ae_enc_conv3_b, 64, 5, 2, out_enc_c3);
    for (int i = 0; i < 64*25; i++) out_enc_c3[i] = relu(out_enc_c3[i]);

    global_avg_pool(out_enc_c3, 64, 25, out_pooled);
    linear(out_pooled, ae_fc_enc_w, ae_fc_enc_b, 64, 16, false, out_latent);

    linear(out_latent, w_fc_dec_w, w_fc_dec_b, 16, 64*25, true, out_dec_fc);
    memcpy(out_dec_c1, out_dec_fc, 64*25*sizeof(float));

    convt1d(out_dec_c1, 64, 25, w_dec_ct1_w, w_dec_ct1_b, 32, 5, 2, 1, out_dec_c2);
    for (int i = 0; i < 32*50; i++) out_dec_c2[i] = relu(out_dec_c2[i]);

    convt1d(out_dec_c2, 32, 50, w_dec_ct2_w, w_dec_ct2_b, 16, 5, 2, 1, out_dec_c3);
    for (int i = 0; i < 16*100; i++) out_dec_c3[i] = relu(out_dec_c3[i]);

    convt1d(out_dec_c3, 16, 100, w_dec_ct3_w, w_dec_ct3_b, 6, 7, 2, 1, output);
}

// ==================== 在线训练 ====================

// ==================== 基线管理 ====================
#define BASELINE_SAMPLES 30
#define BASELINE_RANGE_K 1.0f

static float score_samples[BASELINE_SAMPLES];
static int sample_count = 0;

void ae_update_baseline(int hour, float score, float unused1, float unused2) {
    if (hour < 0 || hour >= 24) return;
    AEHourBaseline* b = &global_baseline.hour[hour];
    
    if (sample_count < BASELINE_SAMPLES) {
        score_samples[sample_count++] = score;
        
        if (sample_count == BASELINE_SAMPLES) {
            float sorted[BASELINE_SAMPLES];
            memcpy(sorted, score_samples, sizeof(sorted));
            sort_floats(sorted, BASELINE_SAMPLES);
            
            b->score_median = sorted[BASELINE_SAMPLES/2];
            float score_min = sorted[0];
            float score_max = sorted[BASELINE_SAMPLES-1];
            
            b->score_lower = b->score_median - (b->score_median - score_min) * BASELINE_RANGE_K;
            b->score_upper = b->score_median + (score_max - b->score_median) * BASELINE_RANGE_K;
            
            b->valid = true;
            sample_count = 0;
            
            Serial.printf("[AE BASELINE] score: [%.3f %.3f %.3f]\n",
                          b->score_lower, b->score_median, b->score_upper);
        }
    }
}
AEBaseline* ae_get_baseline() { 
    return &global_baseline; 
}
void ae_save_baseline() {
    Preferences p;
    p.begin("ae_base", false);
    for (int h = 0; h < 24; h++) {
        char k[16];
        snprintf(k, 16, "v%d", h); p.putBool(k, global_baseline.hour[h].valid);
        if (global_baseline.hour[h].valid) {
            snprintf(k, 16, "sm%d", h); p.putFloat(k, global_baseline.hour[h].score_median);
            snprintf(k, 16, "sl%d", h); p.putFloat(k, global_baseline.hour[h].score_lower);
            snprintf(k, 16, "su%d", h); p.putFloat(k, global_baseline.hour[h].score_upper);
        }
    }
    p.end();
}

void ae_load_baseline() {
    Preferences p;
    p.begin("ae_base", true);
    for (int h = 0; h < 24; h++) {
        char k[16];
        snprintf(k, 16, "v%d", h); 
        global_baseline.hour[h].valid = p.getBool(k, false);
        if (global_baseline.hour[h].valid) {
            snprintf(k, 16, "sm%d", h); global_baseline.hour[h].score_median = p.getFloat(k, 0);
            snprintf(k, 16, "sl%d", h); global_baseline.hour[h].score_lower  = p.getFloat(k, 0);
            snprintf(k, 16, "su%d", h); global_baseline.hour[h].score_upper  = p.getFloat(k, 0);
        }
    }
    p.end();
}


