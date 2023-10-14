#ifndef CAPSTONE
#define CAPSTONE

#include "mbed.h"
#include <cmath>
#include <cstdio>

#define CAN_ID 0x01

#define usingRef 0

#define pi      (3.14159265359f)

#define P_MIN   (-12.5f)
#define P_MAX   (12.5f)
#define V_MIN   (-45.0f)
#define V_MAX   (45.0f)
#define KP_MIN  (0.0f)
#define KP_MAX  (500.0f)
#define KD_MIN  (0.0f)
#define KD_MAX  (5.0f)
#define T_MIN   (-18.0f)
#define T_MAX   (18.0f)
#define I_MAX   (40.0f)

#define dt      0.01

template <typename A> A max(A x, A y) { return x > y ? x : y; }
template <typename A> A min(A x, A y) { return x < y ? x : y; }

struct refs { float p_ref; float v_ref; float kp_ref; float kd_ref; float t_ref; };

enum Mode : int { setzero_mode = 0, runtime_mode = 1, observe_mode = 2, };

extern Serial       pc;
extern CAN          can;
extern Timer        timer;
extern Ticker       sendCAN;
extern int          turn_cnt;
extern struct refs  refs_tbl[1000][3];

extern CANMessage   rxMsg;
extern CANMessage   txMsg1;
extern CANMessage   txMsg2;
extern CANMessage   txMsg3;

void serial_isr(void);
void command(void);
void onMsgReceived(void);
void unpack_reply(CANMessage msg);
void pack_cmd(CANMessage &msg, float p_des, float v_des, float kp, float kd, float t_ff);

inline
void limit_norm(float &x, float &y, float limit)
{   // Scales the lenght of vector (x, y) to be <= limit ///
    float const norm = sqrt(x * x + y * y);
    if(norm > limit) {
        x = x * (limit / norm);
        y = y * (limit / norm);
    }
}

inline
int float_to_uint(float x, float x_min, float x_max, int bits)
{   // Converts a float to an unsigned int, given range and number of bits ///
    float const span = x_max - x_min;
    float const offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}
    
inline
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{   // converts unsigned int to float, given range and number of bits ///
    float const span = x_max - x_min;
    float const offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

#endif
