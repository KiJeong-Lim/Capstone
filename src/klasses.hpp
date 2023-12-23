#ifndef KLASSES_HPP
#define KLASSES_HPP

#include <cstdint>
#include "mbed.h"

typedef enum Mode {
    SetzeroMode = 0,
    RuntimeMode = 1,
    ObserveMode = 2,
    ReadcmdMode = 3,
    GotobedMode = 4,
} Mode_t;

struct MotorInputData {
    float p;
    float v;
    float kp;
    float kd;
    float t_ff;
};

struct MotorOutputData {
    float p;
    float v;
    float i;
};

typedef struct uch8 {
    unsigned char data[8];
} uch8_t;

/* cancomm.cpp */
class MotorInput : public MotorInputData {
private:
    std::uint8_t p_lock_lv;
    std::uint8_t v_lock_lv;
    std::uint8_t kp_lock_lv;
    std::uint8_t kd_lock_lv;
    std::uint8_t t_ff_lock_lv;
public:
    MotorInput();
    MotorInput(const MotorInput &other);
    ~MotorInput();
    void init(void);
    void hexadecimal(const unsigned char (*new_motor_input_data)[8]);
    uch8_t encode16(void) const;
    void pack(CANMessage *can_msg) const;
    void pull(const MotorInputData &new_motor_input_data);
    void set(const MotorInputData &new_motor_input_data);
    bool set_p(float new_p, int key_lv);
    bool set_v(float new_v, int key_lv);
    bool set_kp(float new_kp, int key_lv);
    bool set_kd(float new_kd, int key_lv);
    bool set_t_ff(float new_t_ff, int key_lv);
    void set_p_lock_lv(int new_lock_lv);
    void set_v_lock_lv(int new_lock_lv);
    void set_kp_lock_lv(int new_lock_lv);
    void set_kd_lock_lv(int new_lock_lv);
    void set_t_ff_lock_lv(int new_lock_lv);
    MotorInputData get(void) const;
};

/* cancomm.cpp */
class MotorOutput : public MotorOutputData {
private:
    int id;
public:
    MotorOutput();
    MotorOutput(const MotorOutput &other);
    ~MotorOutput();
    void set_id(int id);
    MotorOutputData get(void) const;
    void unpack(const CANMessage *can_msg);
};

/* pid.cpp */
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

/* utilities.cpp */
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

#endif
