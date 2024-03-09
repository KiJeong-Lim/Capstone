#ifndef CAPSTONE
#define CAPSTONE "V8 double-can 2ed"

#include <cstdio>
#include <cstring>

#include "mbed.h"

#include "changelog.h"

#define USE_PID             false
#define RUNTIME_TICK_MAX    1000000
#define Tick_dt             0.01

#define ESC                 (27)
#define LEFT_DIRECTION      (75)
#define RIGHT_DIRECTION     (77)
#define DEL_KEY             (83)
#define NOT_A_SPECIAL_KEY   (-1)

#define pi          (3.14159265359)

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
#define I_MIN       (-40.0f)
#define I_MAX       (40.0f)

#define len(arr)    ((sizeof(arr)) / (sizeof((arr)[0])))
#define max(x,y)    (((x) >= (y)) ? (x) : (y))
#define min(x,y)    (((y) >= (x)) ? (x) : (y))

#ifndef USE_PID
#define USE_PID     0
#endif

struct UCh8 { unsigned char data[8]; };

typedef enum Mode {
    SetzeroMode = 0,
    RuntimeMode = 1,
    ObserveMode = 2,
    ReadcmdMode = 3,
    SitdownMode = 4,
} Mode_t;

struct GetDataWithId { int motor_id; float p; float v; float i; };

class Motor {
public:
    struct PutData { float p; float v; float kp; float kd; float t_ff; };
    struct GetData { float p; float v; float i; };
public:
    PutData data_into_motor;
    GetData data_from_motor;
    int motor_id;
public:
    void setInputWithHexademical(const UCh8 &encoded_input);
    void pack(CANMessage &can_msg) const;
    void unpack(const CANMessage &can_msg);
};

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
private:
    char buffer[64];
    int cursor;
    int theend;
    char *result;
    void (*prompt)(const char *msg);
public:
    void setPrompt(void (*prompt)(const char *msg));
    bool runPrompt(void);
    static int getc(void);
private:
    bool takech(int ch);
    void print(void) const;
    void clear(void);
    void sync(char *&msg) const;
};

class CANHanlde {
public:
    CAN can;
public:
    CANHanlde(const PinName &rd, const PinName &td);
    void init(unsigned int id, unsigned int mask, void (*to_be_attached)(void));
    void read(CANMessage &rx_msg);
    void write(CANMessage &tx_msg);
};

class MotorHandler : public Motor {
public:
#if USE_PID
    float p_ctrl;
    PIDController pid_on_p;
#endif
    CANMessage tx_msg;
public:
#if USE_PID
    MotorHandler(int id, float Kp, float Ki, float Kd);
#else
    MotorHandler(int id);
#endif
    bool isWellFormed(void) const;
    void putTxMsg(const UCh8 &rhs);
    void sendMsg(void);
    int id(void) const;
#if USE_PID
    bool pidInit(void);
    bool pidCompute(void);
    void set_Kp(float Kp);
    void set_Ki(float Ki);
    void set_Kd(float Kd);
#endif
};

class CANManager {
private:
    CANHanlde can_handle;
    MotorHandler *const *const motor_handlers_vec_ptr;
    const int motor_handlers_vec_size;
    CANMessage rx_msg;
public:
    CANManager(const PinName &rd, const PinName &td, MotorHandler **motor_handlers_vec_ptr, int motor_handlers_vec_size);
    void init(unsigned int id, unsigned int mask, void (*to_be_attached)(void));
    void onMsgReceived(void);
    void sendMsg(void);
};

extern const Motor::PutData reftbl1[1000][3];

extern int      special_key_flag;
extern Serial   pc;
extern Timer    timer;

Motor::PutData  decodetx(const unsigned char (*input_data)[8]);
UCh8            encodetx(const Motor::PutData &input_data);
GetDataWithId   decoderx(const unsigned char *output_data);
void            limitNorm(float &x, float &y, float limit);
unsigned int    float2uint(float x, float x_min, float x_max, int bits);
float           uint2float(unsigned int x_int, float x_min, float x_max, int bits);
float           middle(float x, float y, float z);
double          getTime(void);
bool            areSameStr(const char *lhs, const char *rhs);
bool            inRange(float left, float x, float right);

#endif
