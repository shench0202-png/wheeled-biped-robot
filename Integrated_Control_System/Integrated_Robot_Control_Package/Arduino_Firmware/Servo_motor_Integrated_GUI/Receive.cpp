#include "Receive.h"

static void waitMillisNoDelay(uint32_t wait_ms) {
    uint32_t start_ms = millis();
    while (millis() - start_ms < wait_ms) {
        yield();
    }
}



#if ACTIVE_IMU_SENSOR == IMU_SENSOR_ICM20948
Receive::Receive() : icm(0x68) {
    
}
#else
Receive::Receive() {
    
}
#endif

bool Receive::begin() {
#if ACTIVE_IMU_SENSOR == IMU_SENSOR_ICM20948
    if (!icm.init()) {
        Serial2.println("ICM20948 not detected. Check wiring and I2C address.");
        icm_ready = false;
        mag_detected = false;
        return false;
    }

    icm_ready = true;
    Serial2.println("ICM20948 accel/gyro initialized.");

    mag_detected = icm.initMagnetometer();
    if (mag_detected) {
        icm.setMagOpMode(AK09916_CONT_MODE_100HZ);
        Serial2.println("ICM20948 magnetometer initialized.");
    } else {
        Serial2.println("ICM20948 magnetometer not detected.");
    }

    return true;
#else
    Wire.beginTransmission(MPU_addr);
    if (Wire.endTransmission() != 0) {
        Serial2.println("MPU6050 not detected. Check wiring.");
        mag_detected = false;
        return false;
    }

    Wire.beginTransmission(MPU_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    Serial2.println("MPU6050 detected.");

    Wire.beginTransmission(QMC5883L_addr);
    mag_detected = (Wire.endTransmission() == 0);
    if (mag_detected) {
        qmc_init();
    } else {
        Serial2.println("QMC5883L not detected. Magnetometer update is disabled.");
    }

    return true;
#endif
}


void Receive::Offset() {
    const int n = 100;
    float SumAcc_raw[3] = {0.0f, 0.0f, 0.0f};
    float SumAcc_b[3] = {0.0f, 0.0f, 0.0f};
    float SumAcc_a[3] = {0.0f, 0.0f, 0.0f};
    float SumAcc_ab[3] = {0.0f, 0.0f, 0.0f};
    float SumRollRate_raw = 0, SumPitchRate_raw = 0, SumYawRate_raw = 0;
    float SumRollRate = 0, SumPitchRate = 0, SumYawRate = 0;
    float SumRollRate_a = 0, SumPitchRate_a = 0, SumYawRate_a = 0;
    float SumRollRate_a_b = 0, SumPitchRate_a_b = 0, SumYawRate_a_b = 0;

    for (int axis = 0; axis < 3; axis++) {
        offsetAcc[axis] = 0.0f;
        offset_Aacc[axis] = 0.0f;
        offset_ABacc[axis] = 0.0f;
        offsetAcc_Raw[axis] = 0.0f;
        offsetRate[axis] = 0.0f;
        offset_Arate[axis] = 0.0f;
        offset_ABrate[axis] = 0.0f;
        offsetRate_Raw[axis] = 0.0f;
    }
    resetFilterState();

    Serial2.println("[Offset] Collecting accel/gyro offsets. Keep the robot still...");

    // Same offset strategy as Arduino/IMU/quaternion_new:
    // collect raw Acc/Rate first, then collect each filtered Acc/Rate channel.
    // Acc offsets are direct means, including the static 1g component.
    for (int i = 0; i < n; i++) {
        DataRead(0.01f);
        SumAcc_raw[0] += Acc_raw[0];
        SumAcc_raw[1] += Acc_raw[1];
        SumAcc_raw[2] += Acc_raw[2];
        SumRollRate_raw += Rate_raw[0];
        SumPitchRate_raw += Rate_raw[1];
        SumYawRate_raw += Rate_raw[2];
    }

    offsetAcc_Raw[0] = SumAcc_raw[0] / n;
    offsetAcc_Raw[1] = SumAcc_raw[1] / n;
    offsetAcc_Raw[2] = SumAcc_raw[2] / n;
    offsetRate_Raw[0] = SumRollRate_raw / n;
    offsetRate_Raw[1] = SumPitchRate_raw / n;
    offsetRate_Raw[2] = SumYawRate_raw / n;

    for (int i = 0; i < n; i++) {
        DataRead(0.01f);
        SumAcc_b[0] += Acc_b[0];
        SumAcc_b[1] += Acc_b[1];
        SumAcc_b[2] += Acc_b[2];
        SumRollRate += Rate_b[0];
        SumPitchRate += Rate_b[1];
        SumYawRate += Rate_b[2];

        SumAcc_a[0] += Acc_a[0];
        SumAcc_a[1] += Acc_a[1];
        SumAcc_a[2] += Acc_a[2];
        SumRollRate_a += Rate_a[0];
        SumPitchRate_a += Rate_a[1];
        SumYawRate_a += Rate_a[2];

        SumAcc_ab[0] += Acc_ab[0];
        SumAcc_ab[1] += Acc_ab[1];
        SumAcc_ab[2] += Acc_ab[2];
        SumRollRate_a_b += Rate_ab[0];
        SumPitchRate_a_b += Rate_ab[1];
        SumYawRate_a_b += Rate_ab[2];
    }

    offsetAcc[0] = SumAcc_b[0] / n;
    offsetAcc[1] = SumAcc_b[1] / n;
    offsetAcc[2] = SumAcc_b[2] / n;
    offsetRate[0] = SumRollRate / n;
    offsetRate[1] = SumPitchRate / n;
    offsetRate[2] = SumYawRate / n;

    offset_Aacc[0] = SumAcc_a[0] / n;
    offset_Aacc[1] = SumAcc_a[1] / n;
    offset_Aacc[2] = SumAcc_a[2] / n;
    offset_Arate[0] = SumRollRate_a / n;
    offset_Arate[1] = SumPitchRate_a / n;
    offset_Arate[2] = SumYawRate_a / n;

    offset_ABacc[0] = SumAcc_ab[0] / n;
    offset_ABacc[1] = SumAcc_ab[1] / n;
    offset_ABacc[2] = SumAcc_ab[2] / n;
    offset_ABrate[0] = SumRollRate_a_b / n;
    offset_ABrate[1] = SumPitchRate_a_b / n;
    offset_ABrate[2] = SumYawRate_a_b / n;

    if (mag_detected) {
        float sum_norm = 0.0f;
        int valid = 0;
        for (int i = 0; i < 50; i++) {
            read_magnetometer(0.01f);
            float norm = getMagNorm();
            if (norm > 10 && norm < 1000) {
                sum_norm += norm;
                valid++;
            }
            waitMillisNoDelay(5);
        }
        mag_norm_ref = (valid > 20) ? sum_norm / valid : 0.0f;
        Serial2.print("[Offset] Magnetometer norm reference = ");
        Serial2.println(mag_norm_ref);
    }

    resetFilterState();

    Serial2.print("[Offset] Acc raw offset = ");
    Serial2.print(offsetAcc_Raw[0], 6); Serial2.print(", ");
    Serial2.print(offsetAcc_Raw[1], 6); Serial2.print(", ");
    Serial2.println(offsetAcc_Raw[2], 6);
    Serial2.print("[Offset] Acc butter offset = ");
    Serial2.print(offsetAcc[0], 6); Serial2.print(", ");
    Serial2.print(offsetAcc[1], 6); Serial2.print(", ");
    Serial2.println(offsetAcc[2], 6);
    Serial2.print("[Offset] Acc alpha offset = ");
    Serial2.print(offset_Aacc[0], 6); Serial2.print(", ");
    Serial2.print(offset_Aacc[1], 6); Serial2.print(", ");
    Serial2.println(offset_Aacc[2], 6);
    Serial2.print("[Offset] Acc alpha-beta offset = ");
    Serial2.print(offset_ABacc[0], 6); Serial2.print(", ");
    Serial2.print(offset_ABacc[1], 6); Serial2.print(", ");
    Serial2.println(offset_ABacc[2], 6);
    Serial2.println("[Offset] Accel/gyro offset calculation done.");
}

void Receive::DataRead(float dt) {
    float acc_x_raw = 0.0f;
    float acc_y_raw = 0.0f;
    float acc_z_raw = 0.0f;
    float gyro_x_raw = 0.0f;
    float gyro_y_raw = 0.0f;
    float gyro_z_raw = 0.0f;

#if ACTIVE_IMU_SENSOR == IMU_SENSOR_ICM20948
    if (!icm_ready) {
        mag_ok = false;
        return;
    }

    icm.readSensor();
    xyzFloat acc;
    xyzFloat gyro;
    icm.getGValues(&acc);
    icm.getGyrValues(&gyro);

    acc_x_raw = acc.x;
    acc_y_raw = acc.y;
    acc_z_raw = acc.z;
    gyro_x_raw = gyro.x;
    gyro_y_raw = gyro.y;
    gyro_z_raw = gyro.z;
#else
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x1C);
        Wire.write(0x10);
        Wire.endTransmission();

        Wire.beginTransmission(MPU_addr);
        Wire.write(0x3B);
        Wire.endTransmission();
        Wire.requestFrom(MPU_addr, 6);

        int16_t AccXLSB = Wire.read() << 8 | Wire.read();
        int16_t AccYLSB = Wire.read() << 8 | Wire.read();
        int16_t AccZLSB = Wire.read() << 8 | Wire.read();

        acc_x_raw = (float)AccXLSB / 4096.0f;
        acc_y_raw = (float)AccYLSB / 4096.0f;
        acc_z_raw = (float)AccZLSB / 4096.0f;
#endif

    Butterworth_filter(acc_x_raw, accX);
    Butterworth_filter(acc_y_raw, accY);
    Butterworth_filter(acc_z_raw, accZ);
    Acc_b[0] = accX[0] - offsetAcc[0];
    Acc_b[1] = accY[0] - offsetAcc[1];
    Acc_b[2] = accZ[0] - offsetAcc[2];


    Alpha_filter(acc_x_raw, a_accx);
    Alpha_filter(acc_y_raw, a_accy);
    Alpha_filter(acc_z_raw, a_accz);
    Acc_a[0] = a_accx[0] - offset_Aacc[0];
    Acc_a[1] = a_accy[0] - offset_Aacc[1];
    Acc_a[2] = a_accz[0] - offset_Aacc[2];


    Alpha_Beta_filter(acc_x_raw, ab_accx, abs_accx, dt);
    Alpha_Beta_filter(acc_y_raw, ab_accy, abs_accy, dt);
    Alpha_Beta_filter(acc_z_raw, ab_accz, abs_accz, dt);
    Acc_ab[0] = ab_accx[0] - offset_ABacc[0];
    Acc_ab[1] = ab_accy[0] - offset_ABacc[1];
    Acc_ab[2] = ab_accz[0] - offset_ABacc[2];


    Acc_raw[0] = acc_x_raw - offsetAcc_Raw[0];
    Acc_raw[1] = acc_y_raw - offsetAcc_Raw[1];
    Acc_raw[2] = acc_z_raw - offsetAcc_Raw[2];

#if ACTIVE_IMU_SENSOR != IMU_SENSOR_ICM20948
        Wire.beginTransmission(MPU_addr);
        Wire.write(0x1B);
        Wire.write(0x08);
        Wire.endTransmission();

        Wire.beginTransmission(MPU_addr);
        Wire.write(0x43);
        Wire.endTransmission();
        Wire.requestFrom(MPU_addr, 6);

        int16_t GyroX = Wire.read() << 8 | Wire.read();
        int16_t GyroY = Wire.read() << 8 | Wire.read();
        int16_t GyroZ = Wire.read() << 8 | Wire.read();

        gyro_x_raw = (float)GyroX / 65.5f;
        gyro_y_raw = (float)GyroY / 65.5f;
        gyro_z_raw = (float)GyroZ / 65.5f;
#endif


    Butterworth_filter(gyro_x_raw, gyroX);
    Butterworth_filter(gyro_y_raw, gyroY);
    Butterworth_filter(gyro_z_raw, gyroZ);
    Rate_b[0] = gyroX[0] - offsetRate[0];
    Rate_b[1] = gyroY[0] - offsetRate[1];
    Rate_b[2] = gyroZ[0] - offsetRate[2];


    Alpha_filter(gyro_x_raw, a_ratex);
    Alpha_filter(gyro_y_raw, a_ratey);
    Alpha_filter(gyro_z_raw, a_ratez);
    Rate_a[0] = a_ratex[0] - offset_Arate[0];
    Rate_a[1] = a_ratey[0] - offset_Arate[1];
    Rate_a[2] = a_ratez[0] - offset_Arate[2];


    Alpha_Beta_filter(gyro_x_raw, ab_ratex, abs_ratex, dt);
    Alpha_Beta_filter(gyro_y_raw, ab_ratey, abs_ratey, dt);
    Alpha_Beta_filter(gyro_z_raw, ab_ratez, abs_ratez, dt);
    Rate_ab[0] = ab_ratex[0] - offset_ABrate[0];
    Rate_ab[1] = ab_ratey[0] - offset_ABrate[1];
    Rate_ab[2] = ab_ratez[0] - offset_ABrate[2];


    Rate_raw[0] = gyro_x_raw - offsetRate_Raw[0];
    Rate_raw[1] = gyro_y_raw - offsetRate_Raw[1];
    Rate_raw[2] = gyro_z_raw - offsetRate_Raw[2];

    if (mag_detected) {
        read_magnetometer(dt);
    } else {
        mag_ok = false;
    }

}


void Receive::Receive_get(float Rate[], float Acc[], int choose) {
    if (choose == 1) {
        Rate[0] = Rate_b[0]; Rate[1] = Rate_b[1]; Rate[2] = Rate_b[2];
        Acc[0] = Acc_b[0];   Acc[1] = Acc_b[1];   Acc[2] = Acc_b[2] + 1.0f;
    } else if (choose == 2) {
        Rate[0] = Rate_a[0]; Rate[1] = Rate_a[1]; Rate[2] = Rate_a[2];
        Acc[0] = Acc_a[0];   Acc[1] = Acc_a[1];   Acc[2] = Acc_a[2] + 1.0f;
    } else if (choose == 3) {
        Rate[0] = Rate_ab[0]; Rate[1] = Rate_ab[1]; Rate[2] = Rate_ab[2];
        Acc[0] = Acc_ab[0];   Acc[1] = Acc_ab[1];   Acc[2] = Acc_ab[2] + 1.0f;
    } else if (choose == 4) {
        Rate[0] = Rate_raw[0]; Rate[1] = Rate_raw[1]; Rate[2] = Rate_raw[2];
        Acc[0] = Acc_raw[0];   Acc[1] = Acc_raw[1];   Acc[2] = Acc_raw[2] + 1.0f;
    }
}


float* Receive::Butterworth_filter(float value, float arr[]) {
    arr[3] = value;
    arr[0] = a[0] * arr[1] + a[1] * arr[2] +
             b[0] * arr[3] + b[1] * arr[4] + b[2] * arr[5];
    arr[2] = arr[1];
    arr[1] = arr[0];
    arr[5] = arr[4];
    arr[4] = arr[3];
    return arr;
}


void Receive::Alpha_filter(float raw_data, float data[]) {
    data[0] = data[1] + alpha * (raw_data - data[1]);
    data[1] = data[0];
}


void Receive::Alpha_Beta_filter(float raw_data, float data_one[], float data_two[], float dt) {
    if (dt <= 0) dt = 0.01f;

    float pre_one = data_one[1] + dt * data_two[1];
    float pre_two = data_two[1];
    float r = raw_data - pre_one;

    data_one[0] = pre_one + alpha * r;
    data_two[0] = pre_two + (Beta / dt) * r;

    data_one[1] = data_one[0];
    data_two[1] = data_two[0];
}


void Receive::resetFilterState() {
    memset(accX, 0, sizeof(accX));
    memset(accY, 0, sizeof(accY));
    memset(accZ, 0, sizeof(accZ));
    memset(gyroX, 0, sizeof(gyroX));
    memset(gyroY, 0, sizeof(gyroY));
    memset(gyroZ, 0, sizeof(gyroZ));

    memset(a_accx, 0, sizeof(a_accx));
    memset(a_accy, 0, sizeof(a_accy));
    memset(a_accz, 0, sizeof(a_accz));
    memset(a_ratex, 0, sizeof(a_ratex));
    memset(a_ratey, 0, sizeof(a_ratey));
    memset(a_ratez, 0, sizeof(a_ratez));

    memset(ab_accx, 0, sizeof(ab_accx));
    memset(ab_accy, 0, sizeof(ab_accy));
    memset(ab_accz, 0, sizeof(ab_accz));
    memset(ab_ratex, 0, sizeof(ab_ratex));
    memset(ab_ratey, 0, sizeof(ab_ratey));
    memset(ab_ratez, 0, sizeof(ab_ratez));

    memset(abs_accx, 0, sizeof(abs_accx));
    memset(abs_accy, 0, sizeof(abs_accy));
    memset(abs_accz, 0, sizeof(abs_accz));
    memset(abs_ratex, 0, sizeof(abs_ratex));
    memset(abs_ratey, 0, sizeof(abs_ratey));
    memset(abs_ratez, 0, sizeof(abs_ratez));

    memset(Acc_b, 0, sizeof(Acc_b));
    memset(Acc_a, 0, sizeof(Acc_a));
    memset(Acc_ab, 0, sizeof(Acc_ab));
    memset(Acc_raw, 0, sizeof(Acc_raw));
    memset(Rate_b, 0, sizeof(Rate_b));
    memset(Rate_a, 0, sizeof(Rate_a));
    memset(Rate_ab, 0, sizeof(Rate_ab));
    memset(Rate_raw, 0, sizeof(Rate_raw));
}




void Receive::qmc_init()
{

    Wire.beginTransmission(QMC5883L_addr);
    Wire.write(0x0B);
    Wire.write(0x80);
    Wire.endTransmission();

    waitMillisNoDelay(10);


    Wire.beginTransmission(QMC5883L_addr);
    Wire.write(0x0B);
    Wire.write(0x01);
    Wire.endTransmission();


    Wire.beginTransmission(QMC5883L_addr);
    Wire.write(0x09);
    Wire.write(0x1D);
    Wire.endTransmission();

    waitMillisNoDelay(10);

    Serial2.println("QMC5883L initialized.");
}



void Receive::getCalibratedMag(float mag_out[3], int choose) {
    
    if(choose == 3){
        mag_out[0] = ab_mx[0];
        mag_out[1] = ab_my[0];
        mag_out[2] = ab_mz[0];
    }else if(choose == 4){
        mag_out[0] = Mag_cal[0];
        mag_out[1] = Mag_cal[1];
        mag_out[2] = Mag_cal[2];
    }
     
}


void Receive::getRawMag(float mag_out[3]) {
    mag_out[0] = Mag_raw[0];
    mag_out[1] = Mag_raw[1];
    mag_out[2] = Mag_raw[2];
}


void Receive::applyMagCalibration() {
#if ACTIVE_IMU_SENSOR == IMU_SENSOR_ICM20948
    float norm = sqrtf(Mag_raw[0] * Mag_raw[0] +
                       Mag_raw[1] * Mag_raw[1] +
                       Mag_raw[2] * Mag_raw[2]);
    if (norm > 0.0f) {
        Mag_cal[0] = Mag_raw[0] / norm;
        Mag_cal[1] = Mag_raw[1] / norm;
        Mag_cal[2] = Mag_raw[2] / norm;
    } else {
        Mag_cal[0] = 0.0f;
        Mag_cal[1] = 0.0f;
        Mag_cal[2] = 0.0f;
    }
    return;
#endif

    float ox = Mag_raw[0] - mag_bias[0];
    float oy = Mag_raw[1] - mag_bias[1];
    float oz = Mag_raw[2] - mag_bias[2];


    Mag_cal[0] = ox * mag_soft_iron[0][0] + oy * mag_soft_iron[0][1] + oz * mag_soft_iron[0][2];
    Mag_cal[1] = ox * mag_soft_iron[1][0] + oy * mag_soft_iron[1][1] + oz * mag_soft_iron[1][2];
    Mag_cal[2] = ox * mag_soft_iron[2][0] + oy * mag_soft_iron[2][1] + oz * mag_soft_iron[2][2];
}


bool Receive::qmc_read_raw(float &mx, float &my, float &mz) 
{
#if ACTIVE_IMU_SENSOR == IMU_SENSOR_ICM20948
    if (!icm_ready || !mag_detected) {
        return false;
    }

    icm.readSensor();
    xyzFloat mag;
    icm.getMagValues(&mag);
    mx = mag.x;
    my = mag.y;
    mz = mag.z;
    return true;
#else
    Wire.beginTransmission(QMC5883L_addr);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    if (Wire.requestFrom((uint8_t)QMC5883L_addr, (uint8_t)6) != 6) {
        return false;
    }

    int16_t x = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t y = (int16_t)(Wire.read() | (Wire.read() << 8));
    int16_t z = (int16_t)(Wire.read() | (Wire.read() << 8));

    mx = (float)x;
    my = (float)y;
    mz = (float)z;

    return true;
#endif
}



void Receive::mag_calibrate(uint32_t duration_ms) {
    Serial2.println("\n[CAL] Magnetometer calibration started. Rotate the sensor through all axes.");

    float minv[3] = { 1e9f, 1e9f, 1e9f };
    float maxv[3] = {-1e9f, -1e9f, -1e9f};
    uint32_t t0 = millis();

    while (millis() - t0 < duration_ms) {
        float mx, my, mz;
        if (qmc_read_raw(mx, my, mz)) {
        if (mx < minv[0]) minv[0] = mx; if (mx > maxv[0]) maxv[0] = mx;
        if (my < minv[1]) minv[1] = my; if (my > maxv[1]) maxv[1] = my;
        if (mz < minv[2]) minv[2] = mz; if (mz > maxv[2]) maxv[2] = mz;
        }
        waitMillisNoDelay(20);
    }

    mag_bias[0] = (maxv[0] + minv[0]) * 0.5f;
    mag_bias[1] = (maxv[1] + minv[1]) * 0.5f;
    mag_bias[2] = (maxv[2] + minv[2]) * 0.5f;


    float sx = (maxv[0] - minv[0]) * 0.5f;
    float sy = (maxv[1] - minv[1]) * 0.5f;
    float sz = (maxv[2] - minv[2]) * 0.5f;
    float avg = (sx + sy + sz) / 3.0f;


    for(int i=0; i<3; i++) for(int j=0; j<3; j++) mag_soft_iron[i][j] = (i==j ? 1.0f : 0.0f);
    
    if (sx > 1e-6f) mag_soft_iron[0][0] = avg / sx;
    if (sy > 1e-6f) mag_soft_iron[1][1] = avg / sy;
    if (sz > 1e-6f) mag_soft_iron[2][2] = avg / sz;

    Serial2.println("[CAL] Done. Update mag_bias and mag_soft_iron if needed.");
    Serial2.print("Bias: "); Serial2.print(mag_bias[0]); Serial2.print(", "); Serial2.print(mag_bias[1]); Serial2.print(", "); Serial2.println(mag_bias[2]);

}

bool Receive::read_magnetometer(float dt) {
    if (!mag_detected) {
        mag_ok = false;
        return false;
    }

    float mx, my, mz;
    if (!qmc_read_raw(mx, my, mz)) {
        mag_ok = false;
        return false;
    }

    Mag_raw[0] = mx;
    Mag_raw[1] = my;
    Mag_raw[2] = mz;

    applyMagCalibration();


    float norm = sqrtf(Mag_cal[0]*Mag_cal[0] + 
                       Mag_cal[1]*Mag_cal[1] + 
                       Mag_cal[2]*Mag_cal[2]);


    bool norm_valid = true;
    if (isUsingICM20948()) {
        norm_valid = (norm > 0.05f && norm < 1.5f);
    } else if (mag_norm_ref > 10.0f) {
        float ratio = norm / mag_norm_ref;
        norm_valid = (ratio > (1.0f - mag_norm_tol)) && (ratio < (1.0f + mag_norm_tol));
    } else {

        norm_valid = (norm > 20.0f && norm < 800.0f);
    }


    bool outlier_valid = true;
    for (int i = 0; i < 3; i++) {
        if (isUsingICM20948()) {
            if (fabsf(Mag_cal[i]) > 1.5f) {
                outlier_valid = false;
                break;
            }
        } else if (fabsf(Mag_cal[i]) > 1000.0f || fabsf(Mag_cal[i]) < 5.0f) {
            outlier_valid = false;
            break;
        }
    }


    float gyro_sum = fabsf(Rate_raw[0]) + fabsf(Rate_raw[1]) + fabsf(Rate_raw[2]);
    bool is_still = (gyro_sum < still_gyro_sum_thr);

    if (is_still) {
        mag_still_count = min(mag_still_count + 1, STILL_CONFIRM_N + 5);
    } else {
        mag_still_count = max(mag_still_count - 2, 0);
    }
    bool still_confirmed = (mag_still_count >= STILL_CONFIRM_N);


    mag_ok = norm_valid && outlier_valid && still_confirmed;


    
    Alpha_Beta_filter(Mag_cal[0], ab_mx, abs_mx, dt);
    Alpha_Beta_filter(Mag_cal[1], ab_my, abs_my, dt);
    Alpha_Beta_filter(Mag_cal[2], ab_mz, abs_mz, dt);
    

    return true;
}







float Receive::getMagNorm() const {
    return sqrtf(Mag_cal[0]*Mag_cal[0] + Mag_cal[1]*Mag_cal[1] + Mag_cal[2]*Mag_cal[2]);
}

