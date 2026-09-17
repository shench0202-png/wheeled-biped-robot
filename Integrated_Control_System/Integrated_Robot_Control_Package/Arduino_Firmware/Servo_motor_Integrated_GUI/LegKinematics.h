#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include <Arduino.h>

class LegKinematics {
public:
    // 建構子：初始化機器人尺寸
    LegKinematics(float body_len,
                  float front_big,
                  float front_small,
                  float back_big,
                  float back_small);

    // 設定目標 X
    void setTargetX(float x);

    // 核心：計算左右腿角度
    void angle_cal(float left_height,
                   float right_height,
                   float angle_left[2],
                   float angle_right[2]);

private:
    // 幾何參數
    float body_length;
    float front_big_leg;
    float front_small_leg;
    float back_big_leg;
    float back_small_leg;

    float Target_x;

    // 單腿計算（內部用）
    void calc_single_leg(float height,
                         float &alpha,
                         float &beta);
};

#endif