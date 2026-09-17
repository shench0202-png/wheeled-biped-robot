#ifndef EKF_QUATERNION_H
#define EKF_QUATERNION_H

#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <math.h>
#include <cppQueue.h>

using namespace BLA;

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (3.1415926535f / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / 3.1415926535f)
#endif

struct MagInterferenceCalibration {
    float M0;
    float T_sigma;
    float T_M;
    float T_r;
    float T_E;
    float ref_world[3];
    uint16_t valid_count;
};

class EKF_Quaternion {
private:
    Matrix<4,1> q;        // quaternion state
    Matrix<4,4> P;

    Matrix<4,4> Q;
    Matrix<3,3> R_acc;
    Matrix<3,3> R_mag;

    Matrix<4,4> I;

    // ===== internal =====
    Matrix<4,1> q_pred;
    Matrix<4,4> P_pred;

    // 中間運算的變數
    Matrix<3,3> S;
    Matrix<3,3> S_inv;
    Matrix<3,4> H;
    Matrix<3,1> h;
    Matrix<3,1> z;
    Matrix<4,3> K;
    Matrix<4,1> dq;

    float q0, q1, q2, q3;
    float norm;
    float norm_n; // For 磁力計

    void normalize();
    float wrap_pi(float x);


    // ==================== Envelope 使用 cppQueue ====================
    #define ROLL_WINDOW        300
    #define K_SIGMA            2.0f
    #define SIGMA_FLOOR_ROLL   0.5f
    #define SIGMA_FLOOR_PITCH  0.5f
    #define SIGMA_FLOOR_YAW    2.0f
    #define SIGMA_HIGH_ROLL   15.0f
    #define SIGMA_HIGH_PITCH  15.0f
    #define SIGMA_HIGH_YAW    15.0f

    // 動態縮放包絡線 (防晃機制) 參數
    #define SLOPE_EPS_DEG      0.5f    // 忽略微小雜訊
    #define AMP_REF_DEG        3.0f    // 震幅參考基準
    #define JITTER_HIGH        2.0f    // 晃動分數上限
    #define SHRINK_POWER       2.0f    // 縮放比例的非線性權重
    #define MIN_SHRINK         0.1f    // 最窄縮放倍率

    // 嚴謹狀態機判斷參數
    #define GYRO_QUIET_THRESHOLD 10.0f  // 角速度絕對值總和安全閾值
    #define MOVE_CONFIRM_N   3      // 連續超出 3 筆才算 MOVING
    #define STILL_CONFIRM  15     // 連續在內 15 筆才算 STILL

    #define R0_WINDOW 300
    #define AXIS 3
    // ideal
    // static const int QR_num = 100;
    // R1: Acc, R2: Meg
    cppQueue R1_roll;
    cppQueue R1_pitch;
    cppQueue R1_yaw;
    cppQueue R2_roll;
    cppQueue R2_pitch;
    cppQueue R2_yaw;
    cppQueue Q1_roll;
    cppQueue Q1_pitch;
    cppQueue Q1_yaw;
    float R1[3] = {0};   // for Acc
    float R2[3] = {0};   // for Meg
    float gyro_vars[3] = {0}; // for Gyro


    cppQueue roll_queue;     // 存放 roll (度)
    cppQueue pitch_queue;    // 存放 pitch (度)
    cppQueue yaw_queue;      // 存放 yaw (度)


    // 新增：連續超出計數（debounce）
    int outside_count = 0;
    int inside_count = 0;
    int inside_cnt = 0;
    int gyro_cnt = 0;
    float gyro_abs_sum = 0.0f;
    
    String motion_state = "STILL";  // "STILL" 或 "MOVING"

    float yaw_offset = 0.0f;
    bool yaw_offset_ready = false;
    float current_yaw_raw = 0.0f; // 儲存原始 EKF 輸出的航向

    // ===== Yaw KF =====
    float yaw_kf = 0.0f;
    float P_yaw = 1.0f;
    float Q_yaw = 0.001f;
    float R_yaw = 0.05f;
    cppQueue RKF_yaw;

    // ===== Magnetometer interference rejection =====
    bool mag_interference_rejection_enabled = false;
    bool mag_interfered = false;
    float last_mag_sigma = 0.0f;
    float last_mag_strength_error = 0.0f;
    float last_mag_residual = 0.0f;
    float last_mag_direction_error = 0.0f;
    bool mag_sigma_triggered = false;
    bool mag_strength_triggered = false;
    bool mag_residual_triggered = false;
    bool mag_direction_triggered = false;

    float M0 = 45.0f;
    float ref_world[3] = {1.0f, 0.0f, 0.4f};

    float T_sigma = 2.0f;
    float T_M = 15.0f;
    float T_r = 10.0f;
    float T_E = 0.0872f; // 約 5 度 (sin(5°) ≈ 0.0872)

    float m_filt_prev[3] = {0.0f, 0.0f, 0.0f};
    float v_filt_prev[3] = {0.0f, 0.0f, 0.0f};
    bool mag_filter_ready = false;
    float alpha_m = 0.45f;
    float beta_m = 0.05f;

    cppQueue mag_mag_queue;


    // 輔助函式
    bool calculate_envelope(cppQueue &q_func, float sigma_floor, float sigma_high, float &upper, float &lower);
    void reset_mag_interference_filter();
    void rotate_vector_world_to_body(float vin[3], Matrix<4,1> q_curr, float vout[3]);
    float calculate_mag_std(cppQueue &q_buf);
    bool check_mag_interference(float mag_raw[3], float dt, float &Emk);


public:
    EKF_Quaternion();

    void init();
    void init(bool enable_mag_interference_rejection);
    void init(bool enable_mag_interference_rejection, float mag_reference_strength);

    void update(float gyro[3], float acc[3], float mag[3], float dt);

    void getEuler(float angle[3]);

    // ==================== 新增：Envelope 相關公開函式 ====================
    // == 測有無再動 ==
    void update_envelope_buffer(float roll_deg, float pitch_deg, float yaw_deg);
    bool is_outside_envelope_now(float current_roll_deg, float current_pitch_deg, float current_yaw_deg, float vis_env[2]);
    String getMotionState() const;
    float env_display[2];
    float offset = 0;

    // ==== 更新 R_acc 跟 R_meg ====
    void collect_norm_gyro_acc_meg(float gyro[3], float acc[3], float meg[3], float yaw_meas);
    float compute_variance(cppQueue &q);
    void QR_update(float dt);


    void reset_yaw_offset() { yaw_offset_ready = false; }
    // 取得修正後的航向 (弧度)
    float get_corrected_yaw() {
        float corrected = current_yaw_raw + yaw_offset;
        // Wrap to [-PI, PI]
        while (corrected >  M_PI) corrected -= 2.0f * M_PI;
        while (corrected < -M_PI) corrected += 2.0f * M_PI;
        return corrected * RAD_TO_DEG;
    }
    void get_tilt_compensated_mag_vector(float mag[3], float roll_rad, float pitch_rad, float &hx, float &hy, float &hz);
    void yaw_kf_update(float gyro[3], float mag_raw[3], float yaw_meas, float dt);
    bool isMagInterferenceRejectionEnabled() const { return mag_interference_rejection_enabled; }
    bool isMagInterfered() const { return mag_interfered; }
    float getMagSigma() const { return last_mag_sigma; }
    float getMagStrengthError() const { return last_mag_strength_error; }
    float getMagResidual() const { return last_mag_residual; }
    float getMagDirectionError() const { return last_mag_direction_error; }
    bool isMagSigmaTriggered() const { return mag_sigma_triggered; }
    bool isMagStrengthTriggered() const { return mag_strength_triggered; }
    bool isMagResidualTriggered() const { return mag_residual_triggered; }
    bool isMagDirectionTriggered() const { return mag_direction_triggered; }
    void setMagReferenceStrength(float mag_reference_strength) {
        if (mag_reference_strength > 0.0f) M0 = mag_reference_strength;
    }
    void setMagInterferenceThresholds(float sigma_threshold, float strength_threshold, float residual_threshold, float direction_threshold) {
        if (sigma_threshold > 0.0f) T_sigma = sigma_threshold;
        if (strength_threshold > 0.0f) T_M = strength_threshold;
        if (residual_threshold > 0.0f) T_r = residual_threshold;
        if (direction_threshold > 0.0f) T_E = direction_threshold;
    }
    void setMagReferenceWorld(const float ref[3]) {
        float ref_norm = sqrt(ref[0]*ref[0] + ref[1]*ref[1] + ref[2]*ref[2]);
        if (ref_norm > 0.0f) {
            ref_world[0] = ref[0] / ref_norm;
            ref_world[1] = ref[1] / ref_norm;
            ref_world[2] = ref[2] / ref_norm;
        }
    }
    void applyMagInterferenceCalibration(const MagInterferenceCalibration &cal) {
        setMagReferenceStrength(cal.M0);
        setMagInterferenceThresholds(cal.T_sigma, cal.T_M, cal.T_r, cal.T_E);
        setMagReferenceWorld(cal.ref_world);
        reset_mag_interference_filter();
    }
    float getMagReferenceStrength() const { return M0; }
    float getMagSigmaThreshold() const { return T_sigma; }
    float getMagStrengthThreshold() const { return T_M; }
    float getMagResidualThreshold() const { return T_r; }
    float getMagDirectionThreshold() const { return T_E; }
    float yaw_meas;

};

#endif
