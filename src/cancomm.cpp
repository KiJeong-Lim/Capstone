#include "capstone.h"

MotorInputData decode16(const unsigned char (*const input_data)[8])
{
    const unsigned char *const lines = *input_data;

    const int p_int  = (lines[0] << 8) | (lines[1]);
    const int v_int  = (lines[2] << 4) | (lines[3] >> 4);
    const int kp_int = ((lines[3] & 0x0F) << 8) | (lines[4]);
    const int kd_int = (lines[5] << 4) | (lines[6] >> 4);
    const int t_int  = ((lines[6] & 0x0F) << 8) | (lines[7]);

    const MotorInputData res = {
        .p    = uintToFloat(p_int, P_MIN, P_MAX, 16),
        .v    = uintToFloat(v_int, V_MIN, V_MAX, 12),
        .kp   = uintToFloat(kp_int, KP_MIN, KP_MAX, 12),
        .kd   = uintToFloat(kd_int, KD_MIN, KD_MAX, 12),
        .t_ff = uintToFloat(t_int, T_MIN, T_MAX, 12),
    };

    return res;
}

MotorInput::MotorInput()
    : MotorInputData()
{
    const MotorInputData zero_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };

    this->init();
    this->set(zero_data);
}

MotorInput::MotorInput(const MotorInput &other)
{
    p    = other.p;
    v    = other.v;
    kp   = other.kp;
    kd   = other.kd;
    t_ff = other.t_ff;

    p_lock_lv    = other.p_lock_lv;
    v_lock_lv    = other.v_lock_lv;
    kp_lock_lv   = other.kp_lock_lv;
    kd_lock_lv   = other.kd_lock_lv;
    t_ff_lock_lv = other.t_ff_lock_lv;
}

MotorInput::~MotorInput()
{
}

void MotorInput::init()
{
    p_lock_lv    = 0;
    v_lock_lv    = 0;
    kp_lock_lv   = 0;
    kd_lock_lv   = 0;
    t_ff_lock_lv = 0;
}

void MotorInput::hexadecimal(const unsigned char (*const new_motor_input_data)[8])
{
    const MotorInputData datum = decode16(new_motor_input_data);
    this->set(datum);
}

uch8_t MotorInput::encode16() const
{
    uch8_t res = { .data = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, } };
    
    // limit data to be within bounds
    const float p_des    = middle(P_MIN, p, P_MAX);                  
    const float v_des    = middle(V_MIN, v, V_MAX);
    const float kp_des   = middle(KP_MIN, kd, KP_MAX);
    const float kd_des   = middle(KD_MIN, kd, KD_MAX);
    const float t_ff_des = middle(T_MIN, t_ff, T_MAX);
    // convert floats to unsigned ints
    const unsigned int p_int    = floatToUint(p_des, P_MIN, P_MAX, 16);
    const unsigned int v_int    = floatToUint(v_des, V_MIN, V_MAX, 12);
    const unsigned int kp_int   = floatToUint(kp_des, KP_MIN, KP_MAX, 12);
    const unsigned int kd_int   = floatToUint(kd_des, KD_MIN, KD_MAX, 12);
    const unsigned int t_ff_int = floatToUint(t_ff_des, T_MIN, T_MAX, 12);
    // pack ints into the can buffer
    res.data[0] = p_int >> 8;
    res.data[1] = p_int & 0xFF;
    res.data[2] = v_int >> 4;
    res.data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    res.data[4] = kp_int & 0xFF;
    res.data[5] = kd_int >> 4;
    res.data[6] = ((kd_int & 0xF) << 4) | (t_ff_int >> 8);
    res.data[7] = t_ff_int & 0xff;

    return res;
}

void MotorInput::pack(CANMessage *const can_msg) const
{
    const uch8_t msg = this->encode16();

    for (int i = 0; i < len(can_msg->data); i++) {
        can_msg->data[i] = msg.data[i];
    }
}

void MotorInput::pull(const MotorInputData &new_motor_input_data)
{
    p    = new_motor_input_data.p;
    v    = new_motor_input_data.v;
    kp   = new_motor_input_data.kp;
    kd   = new_motor_input_data.kd;
    t_ff = new_motor_input_data.t_ff;
}

void MotorInput::set(const MotorInputData &new_motor_input_data)
{
    if (p_lock_lv == 0) {
        p = new_motor_input_data.p;
    }
    if (v_lock_lv == 0) {
        v = new_motor_input_data.v;
    }
    if (kp_lock_lv == 0) {
        kp = new_motor_input_data.kp;
    }
    if (kd_lock_lv == 0) {
        kd = new_motor_input_data.kd;
    }
    if (t_ff_lock_lv == 0) {
        t_ff = new_motor_input_data.t_ff;
    }
}

bool MotorInput::set_p(const float p, const int key_lv)
{
    if (p_lock_lv < key_lv) {
        this->p = p;
        return true;
    }
    else {
        printf("p locked: lock = %d, key = %d\n", p_lock_lv, key_lv);
        printf("p: %lf <-- %lf\n", this->p, p);
        return false;
    }
}

bool MotorInput::set_v(const float v, const int key_lv)
{
    if (v_lock_lv < key_lv) {
        this->v = v;
        return true;
    }
    else {
        printf("v locked: lock = %d, key = %d\n", v_lock_lv, key_lv);
        printf("v: %lf <-- %lf\n", this->v, v);
        return false;
    }
}

bool MotorInput::set_kp(const float kp, const int key_lv)
{
    if (kp_lock_lv < key_lv) {
        this->kp = kp;
        return true;
    }
    else {
        printf("kp locked: lock = %d, key = %d\n", kp_lock_lv, key_lv);
        printf("kp: %lf <-- %lf\n", this->kp, kp);
        return false;
    }
}

bool MotorInput::set_kd(const float kd, const int key_lv)
{
    if (kd_lock_lv < key_lv) {
        this->kd = kd;
        return true;
    }
    else {
        printf("kd locked: lock = %d, key = %d\n", kd_lock_lv, key_lv);
        printf("kd: %lf <-- %lf\n", this->kd, kd);
        return false;
    }
}

bool MotorInput::set_t_ff(const float t_ff, const int key_lv)
{
    if (t_ff_lock_lv < key_lv) {
        this->t_ff = t_ff;
        return true;
    }
    else {
        printf("t_ff locked: lock = %d, key = %d\n", t_ff_lock_lv, key_lv);
        printf("t_ff: %lf <-- %lf\n", this->t_ff, t_ff);
        return false;
    }
}

void MotorInput::set_p_lock_lv(const int new_lv)
{
    p_lock_lv = new_lv;
}

void MotorInput::set_v_lock_lv(const int new_lv)
{
    v_lock_lv = new_lv;
}

void MotorInput::set_kp_lock_lv(const int new_lv)
{
    kp_lock_lv = new_lv;
}

void MotorInput::set_kd_lock_lv(const int new_lv)
{
    kd_lock_lv = new_lv;
}

void MotorInput::set_t_ff_lock_lv(const int new_lv)
{
    t_ff_lock_lv = new_lv;
}

MotorOutput::MotorOutput()
{
    id = 0;
    p  = 0.0;
    v  = 0.0;
    i  = 0.0;
}

MotorOutput::MotorOutput(const MotorOutput &other)
{
    id = other.id;
    p  = other.p;
    v  = other.v;
    i  = other.i;
}

MotorOutput::~MotorOutput()
{
}

MotorInputData MotorInput::get() const
{
    const MotorInputData data = { .p = p, .v = v, .kp = kp, .kd = kd, .t_ff = t_ff };
    return data;
}

void MotorOutput::set_id(const int id)
{
    this->id = id;
}

MotorOutputData MotorOutput::get() const
{
    const MotorOutputData data = { .p = p, .v = v, .i = i };

    return data;
}

void MotorOutput::unpack(const CANMessage *const can_msg)
{
    // unpack ints from can buffer
    const int id    = can_msg->data[0];
    const int p_int = (can_msg->data[1] << 8) | can_msg->data[2];
    const int v_int = (can_msg->data[3] << 4) | (can_msg->data[4] >> 4);
    const int i_int = ((can_msg->data[4] & 0xF) << 8) | can_msg->data[5];
    // convert ints to floats
    const float p = uintToFloat(p_int, P_MIN, P_MAX, 16);
    const float v = uintToFloat(v_int, V_MIN, V_MAX, 12);
    const float i = uintToFloat(i_int, -I_MAX, I_MAX, 12);
    // update p, v, i
    if (this->id == id) {
        this->p = p;
        this->v = v;
        this->i = i;
    }
}

void onMsgReceived()
{
    can.read(rx_msg);
    for (int i = 0; i < len(mtr_output); i++) {
        mtr_output[i].unpack(&rx_msg);
    }
}
