#include "MagCalibrator.h"

MagCalibrator::MagCalibrator(Receive& imu, EKF_Quaternion& ekf) 
    : _imu(imu), _ekf(ekf) {
    _ready = false;
}

void MagCalibrator::resetAccumulator() {
    accum_sum_norm = 0; accum_sum_norm_sq = 0;
    accum_ref_sum[0] = 0; accum_ref_sum[1] = 0; accum_ref_sum[2] = 0;
    accum_sum_residual = 0; accum_sum_residual_sq = 0;
    accum_sum_dir_err = 0; accum_sum_dir_err_sq = 0;
    accum_valid_count = 0; accum_residual_count = 0; accum_dir_count = 0;
    accum_filter_ready = false;
}

void MagCalibrator::accumulate(float mag[3], float dt) {
    float mag_body[3] = {mag[1], -mag[0], mag[2]};
    float norm = sqrtf(mag_body[0]*mag_body[0] + mag_body[1]*mag_body[1] + mag_body[2]*mag_body[2]);
    
    if (norm > 1.0f) {
        accum_sum_norm += norm;
        accum_sum_norm_sq += norm * norm;
        accum_ref_sum[0] += mag_body[0] / norm;
        accum_ref_sum[1] += mag_body[1] / norm;
        accum_ref_sum[2] += mag_body[2] / norm;
        accum_valid_count++;

        // 殘差累積 (Alpha-Beta 濾波)
        if (!accum_filter_ready) {
            for (int i = 0; i < 3; i++) {
                accum_m_filt[i] = mag_body[i];
                accum_v_filt[i] = 0.0f;
            }
            accum_filter_ready = true;
        } else {
            float res_v[3];
            const float alpha = 0.45f;
            const float beta = 0.05f;
            for (int i = 0; i < 3; i++) {
                float m_pred = accum_m_filt[i] + accum_v_filt[i] * dt;
                res_v[i] = mag_body[i] - m_pred;
                accum_m_filt[i] = m_pred + alpha * res_v[i];
                accum_v_filt[i] = accum_v_filt[i] + (beta / dt) * res_v[i];
            }
            float residual = sqrtf(res_v[0]*res_v[0] + res_v[1]*res_v[1] + res_v[2]*res_v[2]);
            accum_sum_residual += residual;
            accum_sum_residual_sq += residual * residual;
            accum_residual_count++;
        }

        // 方向誤差累積 (需要已有參考模型)
        if (_ready) {
            float obs_n[3] = {mag_body[0]/norm, mag_body[1]/norm, mag_body[2]/norm};
            float ex = obs_n[1]*_active_cal.ref_world[2] - obs_n[2]*_active_cal.ref_world[1];
            float ey = obs_n[2]*_active_cal.ref_world[0] - obs_n[0]*_active_cal.ref_world[2];
            float ez = obs_n[0]*_active_cal.ref_world[1] - obs_n[1]*_active_cal.ref_world[0];
            float dir_err = sqrtf(ex*ex + ey*ey + ez*ez);
            accum_sum_dir_err += dir_err;
            accum_sum_dir_err_sq += dir_err * dir_err;
            accum_dir_count++;
        }
    }
}

void MagCalibrator::updateModelFromAccumulator() {
    if (accum_valid_count < 10) return; 

    MagInterferenceCalibration cand;
    cand.valid_count = accum_valid_count;
    cand.M0 = (accum_sum_norm / accum_valid_count); 
    float n_var = (accum_sum_norm_sq / accum_valid_count) - (cand.M0 * cand.M0);
    float n_std = (n_var > 0.0f) ? sqrtf(n_var) : 0.0f;
    cand.T_sigma = max(0.001f, 3.0f * n_std);
    cand.T_M = max(0.0005f, 1.0f * n_std) + 0.0005f; // 使用者要求每次更新完加 0.002

    float r_norm = sqrtf(accum_ref_sum[0]*accum_ref_sum[0] + accum_ref_sum[1]*accum_ref_sum[1] + accum_ref_sum[2]*accum_ref_sum[2]);
    if (r_norm > 0.0f) {
        cand.ref_world[0] = accum_ref_sum[0] / r_norm;
        cand.ref_world[1] = accum_ref_sum[1] / r_norm;
        cand.ref_world[2] = accum_ref_sum[2] / r_norm;
    }

    if (accum_residual_count > 0) {
        float r_mean = accum_sum_residual / accum_residual_count;
        float r_var = (accum_sum_residual_sq / accum_residual_count) - (r_mean * r_mean);
        float r_std = (r_var > 0.0f) ? sqrtf(r_var) : 0.0f;
        cand.T_r = (r_mean + 3.0f * r_std) + 8.0f;
    } else { cand.T_r = 10.0f; }

    if (accum_dir_count > 0) {
        float d_mean = accum_sum_dir_err / accum_dir_count;
        float d_var = (accum_sum_dir_err_sq / accum_dir_count) - (d_mean * d_mean);
        float d_std = (d_var > 0.0f) ? sqrtf(d_var) : 0.0f;
        cand.T_E = max(0.0872f, d_mean + 3.0f * d_std); // 門檻下限設為 5 度
    } else { cand.T_E = 0.0872f; }

    // 驗證是否接受 (使用者要求解除限制，原本為 20% M0 與 25 度)
    applyModel(cand, "MAG_LIVE_RECAL");
}

MagInterferenceCalibration MagCalibrator::calibrateInitial(uint16_t sample_count) {
    MagInterferenceCalibration cal;
    const unsigned long interval = 10;
    const float alpha_m = 0.45f;
    const float beta_m = 0.05f;
    const float dt = 0.01f;

    float sum_norm = 0, sum_norm_sq = 0, sum_res = 0, sum_res_sq = 0, ref_sum[3] = {0};
    float m_filt[3] = {0}, v_filt[3] = {0};
    uint16_t valid = 0, res_cnt = 0;
    bool filter_ready = false;

    Serial2.println("MAG_BOOT_CAL_START");
    uint16_t i = 0;
    while (i < sample_count) {
        unsigned long t0 = millis();
        if (_imu.read_magnetometer(0.01f)) {
            float ms[3]; _imu.getCalibratedMag(ms, 4);
            float mb[3] = {ms[1], -ms[0], ms[2]};
            float n = sqrtf(mb[0]*mb[0] + mb[1]*mb[1] + mb[2]*mb[2]);
            if (n > 1.0f) {
                sum_norm += n; sum_norm_sq += n*n;
                ref_sum[0] += mb[0]/n; ref_sum[1] += mb[1]/n; ref_sum[2] += mb[2]/n;
                valid++;
                if (!filter_ready) {
                    for(int j=0; j<3; j++) m_filt[j]=mb[j];
                    filter_ready = true;
                } else {
                    float rv[3];
                    for(int j=0; j<3; j++) {
                        float pr = m_filt[j] + v_filt[j]*dt;
                        rv[j] = mb[j]-pr;
                        m_filt[j] = pr + alpha_m*rv[j];
                        v_filt[j] += (beta_m/dt)*rv[j];
                    }
                    float r = sqrtf(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]);
                    sum_res += r; sum_res_sq += r*r; res_cnt++;
                }
            }
        }
        while(millis()-t0 < interval);
        i++;
    }

    if (valid > 0) {
        cal.M0 = sum_norm / valid;
        float nv = (sum_norm_sq / valid) - (cal.M0 * cal.M0);
        float ns = (nv > 0) ? sqrtf(nv) : 0;
        cal.T_sigma = max(0.001f, 3.0f * ns);
        cal.T_M = max(0.05f, 3.0f * ns);
        float rn = sqrtf(ref_sum[0]*ref_sum[0] + ref_sum[1]*ref_sum[1] + ref_sum[2]*ref_sum[2]);
        cal.ref_world[0] = ref_sum[0]/rn; cal.ref_world[1] = ref_sum[1]/rn; cal.ref_world[2] = ref_sum[2]/rn;
        if (res_cnt > 0) {
            float rm = sum_res/res_cnt;
            float rv = (sum_res_sq/res_cnt) - (rm*rm);
            cal.T_r = (rm + 3.0f*sqrtf(max(0.0f,rv)));
        } else cal.T_r = 10.0f;
        cal.T_E = 0.4f; // 初始預設
        cal.valid_count = valid;
    }
    return cal;
}

void MagCalibrator::applyModel(const MagInterferenceCalibration &cal, const char *tag) {
    _active_cal = cal;
    _ready = true;
    _imu.mag_norm_ref = cal.M0;
    _ekf.applyMagInterferenceCalibration(cal);

    Serial2.print(tag);
    Serial2.print("\tAPPLIED\tM0="); Serial2.print(cal.M0, 4);
    Serial2.print("\tT_M="); Serial2.println(cal.T_M, 4);
}

float MagCalibrator::calculateDirectionError(const float ref_a[3], const float ref_b[3]) {
    float na = sqrtf(ref_a[0]*ref_a[0] + ref_a[1]*ref_a[1] + ref_a[2]*ref_a[2]);
    float nb = sqrtf(ref_b[0]*ref_b[0] + ref_b[1]*ref_b[1] + ref_b[2]*ref_b[2]);
    if (na < 1e-6f || nb < 1e-6f) return 1.0f;
    float ax = ref_a[0]/na, ay = ref_a[1]/na, az = ref_a[2]/na;
    float bx = ref_b[0]/nb, by = ref_b[1]/nb, bz = ref_b[2]/nb;
    float ex = ay*bz - az*by;
    float ey = az*bx - ax*bz;
    float ez = ax*by - ay*bx;
    return sqrtf(ex*ex + ey*ey + ez*ez);
}
