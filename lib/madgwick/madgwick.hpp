#pragma once

namespace madgwick{
    void update_imu(float gx, float gy, float gz, float ax, float ay, float az, float* q);
    void update_marg(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,float* q,float* wb,float* b);
    // void get_quaternion(float* q);
}