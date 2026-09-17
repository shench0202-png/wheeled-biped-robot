#include "EKF_Quaternion.h"

EKF_Quaternion::EKF_Quaternion() 
    : roll_queue(sizeof(float), ROLL_WINDOW, FIFO, true),   // 正確初始化方式
      pitch_queue(sizeof(float), ROLL_WINDOW, FIFO, true),
      yaw_queue(sizeof(float), ROLL_WINDOW, FIFO, true),
      R1_roll(sizeof(float), R0_WINDOW, FIFO, true),
      R1_pitch(sizeof(float), R0_WINDOW, FIFO, true),
      R1_yaw(sizeof(float), R0_WINDOW, FIFO, true),
      R2_roll(sizeof(float), R0_WINDOW, FIFO, true),
      R2_pitch(sizeof(float), R0_WINDOW, FIFO, true),
      R2_yaw(sizeof(float), R0_WINDOW, FIFO, true),
      Q1_roll(sizeof(float), R0_WINDOW, FIFO, true),
      Q1_pitch(sizeof(float), R0_WINDOW, FIFO, true),
      Q1_yaw(sizeof(float), R0_WINDOW, FIFO, true),
      RKF_yaw(sizeof(float), R0_WINDOW, FIFO, true),
      mag_mag_queue(sizeof(float), 50, FIFO, true)
{
    // nothing 
    roll_queue.clean();
    pitch_queue.clean();
    yaw_queue.clean();
    R1_roll.clean();
    R1_pitch.clean();
    R1_yaw.clean();
    R2_roll.clean();
    R2_pitch.clean();
    R2_yaw.clean();
    Q1_roll.clean();
    Q1_pitch.clean();
    Q1_yaw.clean();
    RKF_yaw.clean();
    mag_mag_queue.clean();

    inside_cnt = 0;
    gyro_cnt = 0;
    gyro_abs_sum = 0.0f;
    motion_state = "STILL";

    yaw_offset = 0.0f;
    yaw_offset_ready = false;

}

void EKF_Quaternion::init()
{
    init(false);
}

void EKF_Quaternion::init(bool enable_mag_interference_rejection)
{
    init(enable_mag_interference_rejection, M0);
}

void EKF_Quaternion::init(bool enable_mag_interference_rejection, float mag_reference_strength)
{
    mag_interference_rejection_enabled = enable_mag_interference_rejection;
    setMagReferenceStrength(mag_reference_strength);
    reset_mag_interference_filter();

    q = {1,0,0,0};

    // P = {1,0,0,0,
    //      0,1,0,0,
    //      0,0,1,0,
    //      0,0,0,1};

    P = {1,0.0003,0.0003,0.0003,
         0.0003,1,0.0003,0.0003,
         0.0003,0.0003,1,0.0003,
         0.0003,0.0003,0.0003,1};

    Q.Fill(0.0f);
    for(int i=0;i<4;i++) Q(i,i)= 0.0001;//1e-6;

    R_acc = {0.01,0,0,
             0,0.01,0,
             0,0,0.01};

    R_mag = {0.01,0,0,
             0,0.01,0,
             0,0,0.01};

    

    I.Fill(0.0f);
    for(int i=0;i<4;i++) I(i,i)=1;
}

void EKF_Quaternion::reset_mag_interference_filter()
{
    mag_interfered = false;
    last_mag_sigma = 0.0f;
    last_mag_strength_error = 0.0f;
    last_mag_residual = 0.0f;
    last_mag_direction_error = 0.0f;
    mag_sigma_triggered = false;
    mag_strength_triggered = false;
    mag_residual_triggered = false;
    mag_direction_triggered = false;
    mag_filter_ready = false;
    mag_mag_queue.clean();

    for (int i = 0; i < 3; i++) {
        m_filt_prev[i] = 0.0f;
        v_filt_prev[i] = 0.0f;
    }
}

void EKF_Quaternion::normalize()
{
    float n = sqrt(q(0)*q(0)+q(1)*q(1)+q(2)*q(2)+q(3)*q(3));
    if(n>0) q /= n;
}

float EKF_Quaternion::wrap_pi(float x)
{
    while(x > PI) x -= 2*PI;
    while(x < -PI) x += 2*PI;
    return x;
}

void EKF_Quaternion::update(float gyro[3], float acc[3], float mag[3], float dt)
{

    // 紀錄角速度絕對值總和 (用於運動狀態機雙重保險)
    gyro_abs_sum = fabs(gyro[0]) + fabs(gyro[1]) + fabs(gyro[2]);

    // ======================
    // 1. Prediction
    // ======================
    float wx = gyro[0]*DEG_TO_RAD;
    float wy = gyro[1]*DEG_TO_RAD;
    float wz = gyro[2]*DEG_TO_RAD;

    Matrix<4,4> Omega = {
        0, -wx, -wy, -wz,
        wx, 0, wz, -wy,
        wy, -wz, 0, wx,
        wz, wy, -wx, 0
    };

    Omega *= 0.5f * dt;

    q_pred = (I + Omega) * q;
    P_pred = (I + Omega) * P * ~(I + Omega) + Q;

    // ======================
    // 2. ACC update (roll/pitch)
    // ======================
    float ax = acc[0], ay = acc[1], az = acc[2];
    norm = sqrt(ax*ax + ay*ay + az*az);
    if(norm > 0) {
        ax/=norm; ay/=norm; az/=norm;
    }

    q0=q_pred(0), q1=q_pred(1), q2=q_pred(2), q3=q_pred(3);

    H = {
        -2*q2,  2*q3, -2*q0, 2*q1,
         2*q1,  2*q0,  2*q3, 2*q2,
         2*q0, -2*q1, -2*q2, 2*q3
    };

    h = {
        2*q1*q3 - 2*q0*q2,
        2*q0*q1 + 2*q2*q3,
        q0*q0 - q1*q1 - q2*q2 + q3*q3
    };

    z = {ax, ay, az};

    S = H * P_pred * ~H + R_acc;
    S_inv = S;
    Invert(S_inv);

    K = P_pred * ~H * S_inv;

    dq = K * (z - h);
    dq(3) = 0;   // ❗ 不更新 yaw

    q = q_pred + dq;
    P = (I - K*H) * P_pred;

    // normalize();

    // // ======================
    // // 3. MAG → FULL UPDATE
    // // ======================

    // Keep the raw magnetometer vector for interference detection.
    float raw_meg[3] = {mag[1], -mag[0], mag[2]};

    // normalize magnetometer
    float mx = raw_meg[0];
    float my = raw_meg[1];
    float mz = raw_meg[2];

    norm = sqrt(mx*mx + my*my + mz*mz);
    if(norm > 0){
        mx /= norm;
        my /= norm;
        mz /= norm;
    }
    

    // ===== 地磁向量（世界座標）=====
    // 👉 可以先用這個（台灣近似）
    // float mx_n = 1.0f;
    // float my_n = 0.0f;
    // float mz_n = 0.4f;

    
    // // normalize
    // norm_n = sqrt(mx_n*mx_n + my_n*my_n + mz_n*mz_n);
    // mx_n/=norm_n; my_n/=norm_n; mz_n/=norm_n;

    // // ===== h(q) = R(q) * m_n =====

    // q0 = q(0), q1 = q(1), q2 = q(2), q3 = q(3);

    // // rotation matrix
    // float r11 = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    // float r12 = 2*(q1*q2 + q0*q3);
    // float r13 = 2*(q1*q3 - q0*q2);

    // float r21 = 2*(q1*q2 - q0*q3);
    // float r22 = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    // float r23 = 2*(q2*q3 + q0*q1);

    // float r31 = 2*(q1*q3 + q0*q2);
    // float r32 = 2*(q2*q3 - q0*q1);
    // float r33 = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // // predicted mag
    // h = {
    //     r11*mx_n + r12*my_n + r13*mz_n,
    //     r21*mx_n + r22*my_n + r23*mz_n,
    //     r31*mx_n + r32*my_n + r33*mz_n
    // };

    // // measurement
    // z = {mx, my, mz};

    // // ===== Jacobian =====

    // H = {
    //     2*( q0*mx_n + q3*my_n - q2*mz_n),
    //     2*( q1*mx_n + q2*my_n + q3*mz_n),
    //     2*(-q2*mx_n + q1*my_n - q0*mz_n),
    //     2*(-q3*mx_n + q0*my_n + q1*mz_n),

    //     2*(-q3*mx_n + q0*my_n + q1*mz_n),
    //     2*( q2*mx_n - q1*my_n + q0*mz_n),
    //     2*( q1*mx_n + q2*my_n + q3*mz_n),
    //     2*(-q0*mx_n - q3*my_n + q2*mz_n),

    //     2*( q2*mx_n - q1*my_n + q0*mz_n),
    //     2*( q3*mx_n - q0*my_n - q1*mz_n),
    //     2*( q0*mx_n + q3*my_n - q2*mz_n),
    //     2*( q1*mx_n + q2*my_n + q3*mz_n)
    // };

    // // ===== EKF =====

    // S = H * P * ~H + R_mag;   // ⚠️ 你可以另外設 R_mag
    // S_inv = S;
    // Invert(S_inv);

    // K = P * ~H * S_inv;

    // dq = K * (z - h);
    // dq(1) = 0;
    // dq(2) = 0;

    // // update
    // q = q + dq;

    // // covariance
    // P = (I - K*H) * P;

    normalize();

    float get_gyr[3] = {wx, wy, wz};
    float get_acc[3] = {ax, ay, az};
    // float get_acc[3] = {h(0), h(1), h(2)};
    float get_meg[3] = {mx, my, mz};

    float roll = atan2(2*(q(2)*q(3)+q(0)*q(1)),
                   q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3));
    float pitch = asin(2*(q(0)*q(2)-q(1)*q(3)));
    // float roll = atan2(ay, az);
    // float pitch = atan2(-ax, sqrt(ay*ay + az*az));
    float hx, hy, hz;
    get_tilt_compensated_mag_vector(get_meg, roll, pitch, hx, hy, hz);
    yaw_meas = atan2(-hy, hx);

    collect_norm_gyro_acc_meg(get_gyr, get_acc, get_meg, yaw_meas);

    yaw_kf_update(get_gyr, raw_meg, yaw_meas, dt);

    // offset part 
    // 1. 從當前四元數狀態提取原始航向 (Yaw)
    q0 = q(0,0); q1 = q(1,0); q2 = q(2,0); q3 = q(3,0);
    current_yaw_raw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    // 2. 磁力偏移量捕捉邏輯 (Yaw Offset Logic)
    // 當處於靜止狀態 (STILL) 且尚未完成校準時，紀錄當前航向作為偏移基準
    if (motion_state == "STILL" && !yaw_offset_ready) {
        // 目標是讓目前的 Yaw 變成 0，所以偏移量為負的當前航向
        yaw_offset = -current_yaw_raw;
        yaw_offset_ready = true;
    }

}

void EKF_Quaternion::getEuler(float angle[3])
{
    angle[0] = atan2(2*(q(2)*q(3)+q(0)*q(1)),
                     q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3)) * RAD_TO_DEG;

    angle[1] = asin(2*(q(0)*q(2)-q(1)*q(3))) * RAD_TO_DEG;
    // float sinp = 2 * (q(0) * q(2) - q(1) * q(3));
    // if (fabs(sinp) >= 1)
    //     angle[1] = copysign(M_PI / 2, sinp) * RAD_TO_DEG; // 發生萬向鎖時直接設為 +-90度
    // else
    //     angle[1] = asin(sinp) * RAD_TO_DEG;

    // angle[2] = atan2(2*(q(1)*q(2)+q(0)*q(3)),
    //                  q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3)) * RAD_TO_DEG;

    angle[2] = yaw_kf * RAD_TO_DEG;

    update_envelope_buffer(angle[0], angle[1], angle[2]);
    is_outside_envelope_now(angle[0], angle[1], angle[2], env_display);
    
    // if(angle[0]<1){
    //     angle[2] = angle[2] + angle[0]*(0.5); //- angle[0]*(0.5);
    // }else if(angle[0]>1){
    //     angle[2] = angle[2] - angle[0]*(0.5); //- angle[0]*(0.5);
    // }
    
    


}


bool EKF_Quaternion::calculate_envelope(cppQueue &q_func, float sigma_floor, float sigma_high, float &upper, float &lower) {
    int count = q_func.getCount();
    if (count < 5) {
        return false;
    }

    // 暫存資料陣列
    float data[ROLL_WINDOW];
    for (int i = 0; i < count; i++) {
        q_func.peekIdx(&data[i], i);
    }

    // 第一輪：計算 Mean, Min, Max (取得 P2P 震幅)
    float sum = 0.0f;
    float min_val = data[0];
    float max_val = data[0];
    for (int i = 0; i < count; i++) {
        sum += data[i];
        if (data[i] < min_val) min_val = data[i];
        if (data[i] > max_val) max_val = data[i];
    }
    float mean = sum / count;
    float p2p_amp = max_val - min_val;

    // 第二輪：計算標準差 Std Dev
    float sum_sq = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = data[i] - mean;
        sum_sq += diff * diff;
    }
    float std_dev = sqrt(sum_sq / count);

    // Std 邊界保護
    if (std_dev < sigma_floor) std_dev = sigma_floor;
    if (std_dev > sigma_high) std_dev = sigma_high;

    // 第三輪：計算斜率翻轉率 (Flip Rate)
    int flips = 0;
    int last_slope = 0;
    for (int i = 1; i < count; i++) {
        float diff = data[i] - data[i - 1];
        if (fabs(diff) < SLOPE_EPS_DEG) continue; // 濾除微小雜訊
        
        int current_slope = (diff > 0) ? 1 : -1;
        if (last_slope != 0 && current_slope != last_slope) {
            flips++;
        }
        last_slope = current_slope;
    }
    float flip_rate = (float)flips / count;

    // 第四輪：計算晃動分數 (Jitter Score) 與 縮放因子 (Shrink Factor)
    float jitter_score = flip_rate * (p2p_amp / AMP_REF_DEG);
    float shrink_factor = 1.0f;

    if (jitter_score >= JITTER_HIGH) {
        shrink_factor = MIN_SHRINK; // 晃動嚴重，強制最窄
    } else if (jitter_score > 0.1f) {
        float ratio = 1.0f - (jitter_score / JITTER_HIGH);
        shrink_factor = pow(ratio, SHRINK_POWER);
        if (shrink_factor < MIN_SHRINK) shrink_factor = MIN_SHRINK;
    }

    // 最終 Envelope 計算
    float dynamic_margin = std_dev * K_SIGMA * shrink_factor;
    upper = mean + dynamic_margin;
    lower = mean - dynamic_margin;
    
    return true;
}

// 更新 Envelope（每筆新資料呼叫）
void EKF_Quaternion::update_envelope_buffer(float roll_deg, float pitch_deg, float yaw_deg) {
    float r = roll_deg;
    float p = pitch_deg;
    float y = yaw_deg;

    roll_queue.push(&r);
    pitch_queue.push(&p);
    yaw_queue.push(&y);

    
}


// 主判斷函式：雙重防呆運動狀態機
bool EKF_Quaternion::is_outside_envelope_now(float current_roll_deg, float current_pitch_deg, float current_yaw_deg, float vis_env[2]) {
    float r_upper, r_lower, p_upper, p_lower, y_upper, y_lower;

    bool r_ok = calculate_envelope(roll_queue, SIGMA_FLOOR_ROLL, SIGMA_HIGH_ROLL, r_upper, r_lower);
    bool p_ok = calculate_envelope(pitch_queue, SIGMA_FLOOR_PITCH, SIGMA_HIGH_PITCH, p_upper, p_lower);
    bool y_ok = calculate_envelope(yaw_queue, SIGMA_FLOOR_YAW, SIGMA_HIGH_YAW, y_upper, y_lower);

    if (!r_ok || !p_ok || !y_ok) {
        return false;
    }

    // 條件 1：姿態是否超出動態包絡線
    bool out_roll  = (current_roll_deg  > r_upper) || (current_roll_deg  < r_lower);
    bool out_pitch = (current_pitch_deg > p_upper) || (current_pitch_deg < p_lower);
    bool out_yaw = (current_yaw_deg > y_upper) || (current_yaw_deg < y_lower);
    bool outside_now = out_roll || out_pitch ;//|| out_yaw;

    // 條件 2：角速度是否夠安靜 (利用 update 時算好的 gyro_abs_sum)
    bool is_gyro_quiet = (gyro_abs_sum < GYRO_QUIET_THRESHOLD);

    // 更新狀態機 (雙重保險)
    if (outside_now) {
        // 只要超出通道，無條件立刻切成 MOVING 並歸零所有累積
        motion_state = "MOVING";
        inside_cnt = 0;
        gyro_cnt = 0;
    } else {
        // 在通道內開始累積
        inside_cnt++;

        if (is_gyro_quiet) {
            gyro_cnt++;
        } else {
            gyro_cnt = 0; // 角速度一波動就破功
        }

        // 必須兩者都穩定才回歸 STILL
        if (motion_state == "MOVING") {
            if (inside_cnt >= STILL_CONFIRM && gyro_cnt >= STILL_CONFIRM) {
                motion_state = "STILL";
            }
        }
    }

    env_display[0] = y_upper;
    env_display[1] = y_lower;

    return outside_now;
}

String EKF_Quaternion::getMotionState() const {
    return motion_state;
}

void EKF_Quaternion::collect_norm_gyro_acc_meg(float gyro[3], float acc[3], float meg[3], float yaw_meas)
{
    float r = acc[0];
    float p = acc[1];
    float y = acc[2];

    R1_roll.push(&r);
    R1_pitch.push(&p);
    R1_yaw.push(&y);

    r = meg[0];
    p = meg[1];
    y = meg[2];

    R2_roll.push(&r);
    R2_pitch.push(&p);
    R2_yaw.push(&y);

    r = gyro[0];
    p = gyro[1];
    y = gyro[2];

    Q1_roll.push(&r);
    Q1_pitch.push(&p);
    Q1_yaw.push(&y);

    y = yaw_meas;
    RKF_yaw.push(&y);
    


    // 保持固定長度
    // if (R1_roll.getCount() > R0_WINDOW) {
    //     float dummy;
    //     R1_roll.pop(&dummy);
    //     R1_pitch.pop(&dummy);
    //     R1_yaw.pop(&dummy);
    // }
    // if (R2_roll.getCount() > R0_WINDOW) {
    //     float dummy;
    //     R2_roll.pop(&dummy);
    //     R2_pitch.pop(&dummy);
    //     R2_yaw.pop(&dummy);
    // }

}

float EKF_Quaternion::compute_variance(cppQueue &q)
{
    int N = q.getCount();
    if (N < 10) return 0.0f;

    float mean = 0.0f;
    float val = 0.0f;

    // 計算 mean
    for (int i = 0; i < N; i++) {
        q.peekIdx(&val, i);
        mean += val;
    }
    mean /= N;

    // 計算 variance
    float var = 0.0f;
    for (int i = 0; i < N; i++) {
        q.peekIdx(&val, i);
        float d = val - mean;
        var += d * d;
    }

    var /= N;
    // return var;
    // return (var < 0.00001f) ? 0.00001f : var; // 設定最小值 0.00001(目前OK)
    return (var < 0.0000001f) ? 0.0000001f : var; // 設定最小值 0.00001(目前OK)
}

void EKF_Quaternion::QR_update(float dt){

    float base_r = 0.0007f; // 基本底噪，防止 R 變成 0
    R1[0] = compute_variance(R1_roll);
    R1[1] = compute_variance(R1_pitch);
    R1[2] = compute_variance(R1_yaw);

    R2[0] = compute_variance(R2_roll);
    R2[1] = compute_variance(R2_pitch);
    R2[2] = compute_variance(R2_yaw);

    R_acc = {
        R1[0]+base_r, 0, 0,
        0, R1[1]+base_r, 0,
        0, 0, R1[2]+base_r
    };

    // base_r = 0.001f;
    R_mag = {
        R2[0]+base_r, 0, 0,
        0, R2[1]+base_r, 0,
        0, 0, R2[2]+base_r
    };

    gyro_vars[0] = compute_variance(Q1_roll);
    gyro_vars[1] = compute_variance(Q1_pitch);
    gyro_vars[2] = compute_variance(Q1_yaw);

   
    // gyro covariance
    Matrix<3,3> Qg = {
        gyro_vars[0], 0, 0,
        0, gyro_vars[1], 0,
        0, 0, gyro_vars[2]
    };

    // quaternion
    float qw = q(0), qx = q(1), qy = q(2), qz = q(3);

    // G matrix
    Matrix<4,3> G = {
        -qx, -qy, -qz,
        qw, qz,  -qy,
        -qz,  qw, qx,
        qy,  -qx,  qw
    };

    G *= 0.5f;

    // 正確 Q
    Q = G * Qg * ~G * dt ;

    // 防數值問題
    Q(0,0) += 1e-7;
    Q(1,1) += 1e-7;
    Q(2,2) += 1e-7;
    Q(3,3) += 1e-7;

    float yaw_vars = compute_variance(RKF_yaw);
    // base_r = 0.001f;
    R_yaw = yaw_vars + base_r;
    float base_q = 1*1e-6f;
    Q_yaw = gyro_vars[2] * dt + base_q;
        
}

void EKF_Quaternion::get_tilt_compensated_mag_vector(float mag[3], 
                                                   float roll_rad, float pitch_rad, 
                                                   float &hx, float &hy, float &hz) 
{

    float mx = mag[0]; 
    float my = mag[1]; 
    float mz = mag[2];

    float cr = cos(roll_rad);
    float sr = sin(roll_rad);
    float cp = cos(pitch_rad);
    float sp = sin(pitch_rad);

    // 根據旋轉矩陣 R_y(pitch) * R_x(roll) 的逆運算
    // 將磁力計向量轉回水平面
    hx = mx * cp + my * sr * sp + mz * cr * sp;
    hy = my * cr - mz * sr;
    hz = -mx * sp + my * sr * cp + mz * cr * cp;
}

float EKF_Quaternion::calculate_mag_std(cppQueue &q_buf)
{
    int n = q_buf.getCount();
    if (n < 2) return 0.0f;

    float sum = 0.0f;
    float sq_sum = 0.0f;
    float val = 0.0f;

    for (int i = 0; i < n; i++) {
        q_buf.peekIdx(&val, i);
        sum += val;
        sq_sum += val * val;
    }

    float avg = sum / n;
    float var = (sq_sum / n) - (avg * avg);
    return (var > 0.0f) ? sqrt(var) : 0.0f;
}

void EKF_Quaternion::rotate_vector_world_to_body(float vin[3], Matrix<4,1> q_curr, float vout[3])
{
    float qw = q_curr(0), qx = q_curr(1), qy = q_curr(2), qz = q_curr(3);

    vout[0] = vin[0]*(qw*qw+qx*qx-qy*qy-qz*qz) + vin[1]*(2*(qx*qy+qw*qz))           + vin[2]*(2*(qx*qz-qw*qy));
    vout[1] = vin[0]*(2*(qx*qy-qw*qz))           + vin[1]*(qw*qw-qx*qx+qy*qy-qz*qz) + vin[2]*(2*(qy*qz+qw*qx));
    vout[2] = vin[0]*(2*(qx*qz+qw*qy))           + vin[1]*(2*(qy*qz-qw*qx))           + vin[2]*(qw*qw-qx*qx-qy*qy+qz*qz);
}

bool EKF_Quaternion::check_mag_interference(float mag_raw[3], float dt, float &Emk)
{
    float safe_dt = (dt > 0.0001f) ? dt : 0.0001f;
    float current_M = 0.0f;
    float res_vec[3] = {0.0f, 0.0f, 0.0f};

    if (!mag_filter_ready) {
        for (int i = 0; i < 3; i++) {
            m_filt_prev[i] = mag_raw[i];
            v_filt_prev[i] = 0.0f;
            current_M += mag_raw[i] * mag_raw[i];
        }
        mag_filter_ready = true;
    } else {
        for (int i = 0; i < 3; i++) {
            float m_pred = m_filt_prev[i] + v_filt_prev[i] * safe_dt;
            res_vec[i] = mag_raw[i] - m_pred;
            m_filt_prev[i] = m_pred + alpha_m * res_vec[i];
            v_filt_prev[i] = v_filt_prev[i] + (beta_m / safe_dt) * res_vec[i];
            current_M += mag_raw[i] * mag_raw[i];
        }
    }

    current_M = sqrt(current_M);
    last_mag_residual = sqrt(res_vec[0]*res_vec[0] + res_vec[1]*res_vec[1] + res_vec[2]*res_vec[2]);

    mag_mag_queue.push(&current_M);
    last_mag_sigma = calculate_mag_std(mag_mag_queue);
    last_mag_strength_error = fabs(current_M - M0);

    // 方向檢查 (使用 Roll/Pitch 進行傾斜補償，排除 Yaw 飄移影響)
    float roll_now = atan2(2.0f * (q(2) * q(3) + q(0) * q(1)), q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3));
    float pitch_now = asin(2.0f * (q(0) * q(2) - q(1) * q(3)));
    float hx, hy, hz;
    get_tilt_compensated_mag_vector(mag_raw, roll_now, pitch_now, hx, hy, hz);
    
    // 計算觀測到的垂直分量佔總強度的比例 (即 sin(Dip Angle))
    float obs_dip_sin = (current_M > 0.001f) ? (hz / current_M) : 0.0f;
    // 與參考值比較 (ref_world[2] 為世界座標基準的 Z 分量)
    last_mag_direction_error = fabs(obs_dip_sin - ref_world[2]);

    // 3. 運動狀態下放寬門檻 (Dynamic Thresholding)
    float dynamic_multiplier = (motion_state == "MOVING") ? 2.0f : 1.0f; // 運動時放寬 2 倍

    mag_sigma_triggered = (last_mag_sigma > T_sigma * dynamic_multiplier);
    mag_strength_triggered = (last_mag_strength_error > T_M * dynamic_multiplier);
    mag_residual_triggered = (last_mag_residual > T_r * dynamic_multiplier);
    mag_direction_triggered = (last_mag_direction_error > T_E * dynamic_multiplier);

    return (mag_strength_triggered || mag_residual_triggered || mag_direction_triggered);
}

void EKF_Quaternion::yaw_kf_update(float gyro[3], float mag_raw[3], float yaw_meas, float dt)
{

    float roll = atan2(2*(q(2)*q(3)+q(0)*q(1)),
                   q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3));
    float pitch = asin(2*(q(0)*q(2)-q(1)*q(3)));

    float sp = sinf(roll), cp = cosf(roll);
    float st = sinf(pitch), ct = cosf(pitch);
    float ct_safe = (fabsf(ct) < 0.01f) ? 0.01f : ct;
    float yaw_dot = (gyro[1]*sp + gyro[2]*cp) / ct_safe;
    // float yaw_dot = gyro[2];

    yaw_kf += yaw_dot * dt;
    yaw_kf = wrap_pi(yaw_kf);

    P_yaw += Q_yaw;

    if (mag_interference_rejection_enabled && motion_state != "MOVING") {
        float Emk_val = 0.0f;
        mag_interfered = check_mag_interference(mag_raw, dt, Emk_val);
        if (mag_interfered) {
            return;
        }
    } else {
        // MOVING 狀態或未啟動時，不進行干擾判定，強制 trust 磁力計
        mag_interfered = false;
        mag_sigma_triggered = false;
        mag_strength_triggered = false;
        mag_residual_triggered = false;
        mag_direction_triggered = false;
        
        // 雖然不判定干擾，但仍可選擇性更新內部濾波器 (選用)
        // float Emk_dummy;
        // check_mag_interference(mag_raw, dt, Emk_dummy); 
    }

    float y = wrap_pi(yaw_meas - yaw_kf);

    float K = P_yaw / (P_yaw + R_yaw);

    yaw_kf += K * y;
    yaw_kf = wrap_pi(yaw_kf);

    P_yaw = (1 - K) * P_yaw;
}
