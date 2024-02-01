#ifndef CAPSTONE
#define CAPSTONE "V8-double-can-ver2"

#include <cstdio>
#include <cstring>

#include "mbed.h"

#define VERSION             "0.1.1 (2024-02-02 04:39)"

#define USE_PID             0
#define REF_TBL_ID          0
#define RUNTIME_TICK_MAX    1000000
#define PID_START_TICK      390
#define Tick_dt             0.01
#define DEBUG_TXMSG         false

#define ESC 	            27
#define LEFT_DIRECTION      75
#define RIGHT_DIRECTION     77

#define pi          (3.14159265359f)

#define P_MIN       (-12.5f)
#define P_MAX       (12.5f)
#define V_MIN       (-45.0f)
#define V_MAX       (45.0f)
#define KP_MIN      (0.0f)
#define KP_MAX      (500.0f)
#define KD_MIN      (0.0f)
#define KD_MAX      (5.0f)
#define T_MIN       (-18.0f)
#define T_MAX       (18.0f)
#define I_MAX       (40.0f)

#define len(xs)     (sizeof(xs)/sizeof*(xs))
#define max(x,y)    ((x) >= (y) ? (x) : (y))
#define min(x,y)    ((y) >= (x) ? (x) : (y))

struct UCh8 { unsigned char data[8]; };

class Motor {
public:
    struct SetData { float p; float v; float kp; float kd; float t_ff; };
    struct GetData { float p; float v; float i; };
public:
    SetData data_to_motor;
    GetData data_from_motor;
    int motor_id;
public:
    void setInputWithHexademical(const UCh8 &encoded_input);
    UCh8 encode16(void) const;
    void pack(CANMessage *can_msg) const;
    void unpack(const CANMessage *can_msg);
};

typedef enum Mode {
    SetzeroMode = 0,
    RuntimeMode = 1,
    ObserveMode = 2,
    ReadcmdMode = 3,
    SitdownMode = 4,
} Mode_t;

class PIDController {
public:
    typedef float Real_t;
private:
    Real_t last_time;
    Real_t last_error;
    Real_t error_sum;
public:
    Real_t Kp;
    Real_t Ki;
    Real_t Kd;
    volatile Real_t *PV;
    volatile Real_t *MV;
    volatile Real_t *SP;
    Real_t MV_MIN;
    Real_t MV_MAX;
public:
    PIDController(Real_t Kp, Real_t Ki, Real_t Kd, volatile Real_t *PV, volatile Real_t *MV, volatile Real_t *SP, Real_t MV_MIN, Real_t MV_MAX);
    bool init(void);
    bool compute(void);
    Real_t get_last_time(void) const;
    Real_t get_last_error(void) const;
    Real_t get_error_sum(void) const;
};

class Gear {
public:
    int gear;
private:
    int gear_cnt;
public:
    Gear(int gear);
    Gear(const Gear &other);
    ~Gear();
    bool go(void);
    void reset(void);
};

class IO {
    char buffer[64];
    int cursor;
    int theend;
    char *result;
    void (*prompt)(const char *msg);
public:
    void set_prompt(void (*prompt)(const char *msg));
    bool run_prompt(void);
    static int getc(void);
private:
    bool takech(int ch);
    void print(void);
    void clear(void);
    void sync(char *&msg);
};

class CANHelper {
public:
    CAN my_can;
public:
    CANHelper(PinName rd, PinName td);
    void init(unsigned int id, unsigned int mask, void (*to_be_attached)(void));
    void read(CANMessage &msg);
};

extern Serial   pc;
extern Timer    timer;

extern const Motor::SetData ref_tbl[1000][3];

Motor::SetData  decode16(const unsigned char (*input_data)[8]);
int             main(void);
void            limitNorm(float &x, float &y, float limit);
unsigned int    floatToUint(float x, float x_min, float x_max, int bits);
float           uintToFloat(int x_int, float x_min, float x_max, int bits);
float           middle(float x, float y, float z);
float           getTime(void);
bool            areSameStr(const char *lhs, const char *rhs);
bool            inRange(float left, float x, float rhs);

#endif
