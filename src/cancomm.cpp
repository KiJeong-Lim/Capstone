#include "capstone.hpp"

Motor::PutData decodetx(const unsigned char (*const data_into_motor)[8])
{
    const unsigned char *const lines = *data_into_motor;

    const unsigned int p_int  = (lines[0] << 8) | (lines[1]);
    const unsigned int v_int  = (lines[2] << 4) | (lines[3] >> 4);
    const unsigned int kp_int = ((lines[3] & 0x0F) << 8) | (lines[4]);
    const unsigned int kd_int = (lines[5] << 4) | (lines[6] >> 4);
    const unsigned int t_int  = ((lines[6] & 0x0F) << 8) | (lines[7]);

    const Motor::PutData res = {
        .p    = uint2float(p_int, P_MIN, P_MAX, 16),
        .v    = uint2float(v_int, V_MIN, V_MAX, 12),
        .kp   = uint2float(kp_int, KP_MIN, KP_MAX, 12),
        .kd   = uint2float(kd_int, KD_MIN, KD_MAX, 12),
        .t_ff = uint2float(t_int, T_MIN, T_MAX, 12),
    };

    return res;
}

UCh8 encodetx(const Motor::PutData &data_into_motor)
{
    UCh8 res = { .data = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, } };

    const float p_des    = middle(P_MIN, data_into_motor.p, P_MAX);
    const float v_des    = middle(V_MIN, data_into_motor.v, V_MAX);
    const float kp_des   = middle(KP_MIN, data_into_motor.kd, KP_MAX);
    const float kd_des   = middle(KD_MIN, data_into_motor.kd, KD_MAX);
    const float t_ff_des = middle(T_MIN, data_into_motor.t_ff, T_MAX);

    const unsigned int p_int    = float2uint(p_des, P_MIN, P_MAX, 16);
    const unsigned int v_int    = float2uint(v_des, V_MIN, V_MAX, 12);
    const unsigned int kp_int   = float2uint(kp_des, KP_MIN, KP_MAX, 12);
    const unsigned int kd_int   = float2uint(kd_des, KD_MIN, KD_MAX, 12);
    const unsigned int t_ff_int = float2uint(t_ff_des, T_MIN, T_MAX, 12);

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

GetDataWithId decoderx(const unsigned char *const data)
{
    const unsigned int id    = data[0];
    const unsigned int p_int = (data[1] << 8) | data[2];
    const unsigned int v_int = (data[3] << 4) | (data[4] >> 4);
    const unsigned int i_int = ((data[4] & 0x0F) << 8) | data[5];

    const float p = uint2float(p_int, P_MIN, P_MAX, 16);
    const float v = uint2float(v_int, V_MIN, V_MAX, 12);
    const float i = uint2float(i_int, I_MIN, I_MAX, 12);

    const GetDataWithId res = { .motor_id = id, .p = p, .v = v, .i = i, };

    return res;
}

void Motor::setInputWithHexademical(const UCh8 &encoded_input)
{
    this->data_into_motor = decodetx(&encoded_input.data);
}

void Motor::pack(CANMessage &can_msg) const
{
    const UCh8 msg = encodetx(data_into_motor);

    for (int i = 0; i < len(can_msg.data); i++) {
        can_msg.data[i] = msg.data[i];
    }
}

void Motor::unpack(const CANMessage &can_msg)
{
    const GetDataWithId data_from_motor = decoderx(can_msg.data);

    if (this->motor_id == data_from_motor.motor_id) {
        this->data_from_motor.p = data_from_motor.p;
        this->data_from_motor.v = data_from_motor.v;
        this->data_from_motor.i = data_from_motor.i;
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
