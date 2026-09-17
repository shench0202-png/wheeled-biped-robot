#ifndef MAG_CALIBRATOR_H
#define MAG_CALIBRATOR_H

#include <Arduino.h>
#include "EKF_Quaternion.h"
#include "Receive.h"

// 磁力計干擾模型更新與累積器類別
class MagCalibrator {
public:
    MagCalibrator(Receive& imu, EKF_Quaternion& ekf);

    // 重置累積器
    void resetAccumulator();

    // 累積單次數據 (在 STILL 狀態下呼叫)
    void accumulate(float mag[3], float dt);

    // 執行實時更新 (從累積數據計算並套用)
    void updateModelFromAccumulator();

    // 初始校準 (阻塞式，用於 setup)
    MagInterferenceCalibration calibrateInitial(uint16_t sample_count);

    // 套用校準模型
    void applyModel(const MagInterferenceCalibration &cal, const char *tag);

    // 取得當前模型
    const MagInterferenceCalibration& getActiveCal() const { return _active_cal; }
    bool isReady() const { return _ready; }

private:
    Receive& _imu;
    EKF_Quaternion& _ekf;

    MagInterferenceCalibration _active_cal;
    bool _ready = false;

    // 累積變數
    float accum_sum_norm = 0, accum_sum_norm_sq = 0;
    float accum_ref_sum[3] = {0, 0, 0};
    float accum_sum_residual = 0, accum_sum_residual_sq = 0;
    float accum_sum_dir_err = 0, accum_sum_dir_err_sq = 0;
    int accum_valid_count = 0, accum_residual_count = 0, accum_dir_count = 0;
    
    bool accum_filter_ready = false;
    float accum_m_filt[3] = {0}, accum_v_filt[3] = {0};

    // 輔助運算
    float calculateDirectionError(const float ref_a[3], const float ref_b[3]);
};

#endif
