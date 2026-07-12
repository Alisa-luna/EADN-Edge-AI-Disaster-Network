#ifndef AE_MODEL_H
#define AE_MODEL_H

#include <stdint.h>

// ==================== 网络结构 ====================
#define AE_CHANNELS      6
#define AE_WINDOW        200
#define AE_LATENT        16

// ==================== 输出结构 ====================
typedef struct {
    float mse;
    float latent_dist;
    float spectral_err;
    float score;
} AEOutput;

// ==================== 基线结构（简单版） ====================
// 基线结构改为聚类统计

typedef struct {
    float score_median;
    float score_lower;
    float score_upper;
    bool valid;
} AEHourBaseline;

typedef struct {
    AEHourBaseline hour[24];
} AEBaseline;

// ==================== 函数声明 ====================
void ae_init_weights();
AEOutput ae_predict(const float* input, float* output, float* latent);
void ae_update_baseline(int hour, float mse, float lat, float spec);
float ae_get_threshold(int hour);
void ae_load_baseline();
void ae_save_baseline();
AEBaseline* ae_get_baseline();
void ae_train_step(const float* input, float lr);
void ae_train_step_full(const float* input, float lr);
void ae_reset_weights_to_original();  // 每个小时开始时重置

#endif