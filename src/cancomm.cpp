#include "capstone.hpp"

Motor::PutData decode16(const unsigned char (*const encoded_data)[8])
{
    const unsigned char *const lines = *encoded_data;

    const int p_int  = (lines[0] << 8) | (lines[1]);
    const int v_int  = (lines[2] << 4) | (lines[3] >> 4);
    const int kp_int = ((lines[3] & 0x0F) << 8) | (lines[4]);
    const int kd_int = (lines[5] << 4) | (lines[6] >> 4);
    const int t_int  = ((lines[6] & 0x0F) << 8) | (lines[7]);

    const Motor::PutData res = {
        .p    = int2float(p_int, P_MIN, P_MAX, 16),
        .v    = int2float(v_int, V_MIN, V_MAX, 12),
        .kp   = int2float(kp_int, KP_MIN, KP_MAX, 12),
        .kd   = int2float(kd_int, KD_MIN, KD_MAX, 12),
        .t_ff = int2float(t_int, T_MIN, T_MAX, 12),
    };

    return res;
}

UCh8 encode16(const Motor::PutData &data_into_motor)
{
    UCh8 res = { .data = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, } };

    const float p_des    = middle(P_MIN, data_into_motor.p, P_MAX);
    const float v_des    = middle(V_MIN, data_into_motor.v, V_MAX);
    const float kp_des   = middle(KP_MIN, data_into_motor.kd, KP_MAX);
    const float kd_des   = middle(KD_MIN, data_into_motor.kd, KD_MAX);
    const float t_ff_des = middle(T_MIN, data_into_motor.t_ff, T_MAX);

    const int p_int    = float2int(p_des, P_MIN, P_MAX, 16);
    const int v_int    = float2int(v_des, V_MIN, V_MAX, 12);
    const int kp_int   = float2int(kp_des, KP_MIN, KP_MAX, 12);
    const int kd_int   = float2int(kd_des, KD_MIN, KD_MAX, 12);
    const int t_ff_int = float2int(t_ff_des, T_MIN, T_MAX, 12);

    res.data[0] = p_int >> 8;
    res.data[1] = p_int & 0xFF;
    res.data[2] = v_int >> 4;
    res.data[3] = ((v_int & 0x0F) << 4) | (kp_int >> 8);
    res.data[4] = kp_int & 0xFF;
    res.data[5] = kd_int >> 4;
    res.data[6] = ((kd_int & 0x0F) << 4) | (t_ff_int >> 8);
    res.data[7] = t_ff_int & 0xFF;

    return res;
}

void Motor::setInputWithHexademical(const UCh8 &encoded_input)
{
    this->data_into_motor = decode16(&encoded_input.data);
}

void Motor::pack(CANMessage &can_msg) const
{
    const UCh8 msg = encode16(data_into_motor);

    for (int i = 0; i < len(can_msg.data); i++) {
        can_msg.data[i] = msg.data[i];
    }
}

void Motor::unpack(const CANMessage &can_msg)
{
    // unpack unsigned ints from can buffer
    const unsigned int id    = can_msg.data[0];
    const unsigned int p_int = (can_msg.data[1] << 8) | can_msg.data[2];
    const unsigned int v_int = (can_msg.data[3] << 4) | (can_msg.data[4] >> 4);
    const unsigned int i_int = ((can_msg.data[4] & 0x0F) << 8) | can_msg.data[5];
    // convert unsigned ints to floats
    const float p = int2float(p_int, P_MIN, P_MAX, 16);
    const float v = int2float(v_int, V_MIN, V_MAX, 12);
    const float i = int2float(i_int, I_MIN, I_MAX, 12);
    // update p, v, i
    if (this->motor_id == id) {
        this->data_from_motor.p = p;
        this->data_from_motor.v = v;
        this->data_from_motor.i = i;
    }
}

CANHanlde::CANHanlde(const PinName &rd, const PinName &td)
    : can(rd, td)
{
}

void CANHanlde::init(const unsigned int id, const unsigned int mask, void (*const to_be_attached)(void))
{
    can.frequency(1000000);
    can.attach(to_be_attached);
    can.filter(id, mask, CANStandard, 0);
}

void CANHanlde::read(CANMessage &rx_msg)
{
    can.read(rx_msg);
}

void CANHanlde::write(CANMessage &tx_msg)
{
    can.write(tx_msg);
}
