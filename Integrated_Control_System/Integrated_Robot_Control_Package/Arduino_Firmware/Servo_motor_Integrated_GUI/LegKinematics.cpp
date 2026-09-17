#include "LegKinematics.h"
#include <math.h>

LegKinematics::LegKinematics(float body_len,
                             float front_big,
                             float front_small,
                             float back_big,
                             float back_small)
{
    body_length = body_len;
    front_big_leg = front_big;
    front_small_leg = front_small;
    back_big_leg = back_big;
    back_small_leg = back_small;

    Target_x = body_length / 2.0f;
}

void LegKinematics::setTargetX(float x) {
    Target_x = x;
}


void LegKinematics::angle_cal(float left_height,
                             float right_height,
                             float angle_left[2],
                             float angle_right[2])
{
    calc_single_leg(left_height, angle_left[0], angle_left[1]);
    calc_single_leg(right_height, angle_right[0], angle_right[1]);
}

void LegKinematics::calc_single_leg(float height,
                                    float &alpha,
                                    float &beta)
{
    float a, b, c;
    float d, e, f;
    float determinant;

    // ===== Alpha =====
    a = 2 * Target_x * front_big_leg;
    b = 2 * height * front_big_leg;
    c = pow(Target_x, 2) + pow(height, 2)
        + pow(front_big_leg, 2)
        - pow(front_small_leg, 2);

    determinant = pow(a, 2) + pow(b, 2) - pow(c, 2);

    if (determinant >= 0 && (a + c) != 0) {
        float val = (b + sqrt(determinant)) / (a + c);
        alpha = 2 * atan(val) * 180.0f / PI;
    } else {
        alpha = 0; // fallback
    }

    // ===== Beta =====
    d = 2 * (Target_x - body_length) * front_big_leg;
    e = 2 * height * front_big_leg;
    f = pow(Target_x - body_length, 2)
        + pow(front_big_leg, 2)
        + pow(height, 2)
        - pow(front_small_leg, 2);

    determinant = pow(d, 2) + pow(e, 2) - pow(f, 2);

    if (determinant >= 0 && (d + f) != 0) {
        float val = (e + sqrt(determinant)) / (d + f);
        beta = 2 * atan(val) * 180.0f / PI;
    } else {
        beta = 0;
    }
}