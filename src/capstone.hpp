#ifndef CAPSTONE_HPP
#define CAPSTONE_HPP

#include <cmath>
#include "mbed.h"

#define PI 3.14159265359f
#define CAN_ID 0x01
#define USE_PID true

float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void limit_norm(float *x, float *y, float limit);
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
float middle(float x, float y, float z);
float getTime(void);

struct UCh8 { unsigned char data[8]; };

class PIDController {
public:
    typedef volatile float Real_t;
private:
    Real_t last_time;
    Real_t last_error;
    Real_t error_sum;
public:
    Real_t Kp;
    Real_t Ki;
    Real_t Kd;
    Real_t *PV;
    Real_t *MV;
    Real_t *SP;
    const Real_t MV_MIN;
    const Real_t MV_MAX;
public:
    PIDController(Real_t Kp, Real_t Ki, Real_t Kd, Real_t *PV, Real_t *MV, Real_t *SP, Real_t MV_MIN, Real_t MV_MAX);
    bool init(void);
    bool compute(void);
    Real_t get_last_time(void) const;
    Real_t get_last_error(void) const;
    Real_t get_error_sum(void) const;
};

struct PutData {
    float p;
    float v;
    float kp;
    float kd;
    float t_ff;
};

#endif
