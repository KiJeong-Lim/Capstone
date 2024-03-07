#include "capstone.hpp"

#if USE_PID
MotorHandler::MotorHandler(const int id, const float Kp, const float Ki, const float Kd)
    : p_ctrl(0.0), pid_on_p(Kp, Ki, Kd, &data_from_motor.p, &p_ctrl, &data_into_motor.p, P_MIN, P_MAX)
{
    tx_msg.len = 8;
    tx_msg.id = id;
    this->motor_id = id;
}
#else
MotorHandler::MotorHandler(const int id)
{
    tx_msg.len = 8;
    tx_msg.id = id;
    this->motor_id = id;
}
#endif

bool MotorHandler::isWellFormed() const
{
    return tx_msg.len == 8 && tx_msg.id == motor_id;
}

void MotorHandler::putTxMsg(const UCh8 &rhs)
{
    for (int i = 0; i < len(tx_msg.data); i++) {
        tx_msg.data[i] = rhs.data[i];
    }
}

void MotorHandler::sendMsg()
{
    this->pack(tx_msg);
}

int MotorHandler::id() const
{
    return tx_msg.id;
}

#if USE_PID
bool MotorHandler::pidInit()
{
    bool okay = pid_on_p.init();
    if (!okay) {
        printf("\rFailed to initialize the PID controller of the motor #%d...\n", motor_id);
    }
    return okay;
}

bool MotorHandler::pidCompute()
{
    bool okay = pid_on_p.compute();
    if (okay) {
        data_into_motor.p = p_ctrl;
    }
    return okay;
}

void MotorHandler::set_Kp(const float Kp)
{
    pid_on_p.Kp = Kp;
}

void MotorHandler::set_Ki(const float Ki)
{
    pid_on_p.Ki = Ki;
}

void MotorHandler::set_Kd(const float Kd)
{
    pid_on_p.Kd = Kd;
}
#endif

CANManager::CANManager(const PinName &rd, const PinName &td, MotorHandler **const motor_handlers_vec_ptr, const int motor_handlers_vec_size)
    : can_handle(rd, td), motor_handlers_vec_ptr(motor_handlers_vec_ptr), motor_handlers_vec_size(motor_handlers_vec_size)
{
    rx_msg.len = 6;
}

void CANManager::init(const unsigned int id, const unsigned int mask, void (*const to_be_attached)(void))
{
    can_handle.init(id, mask, to_be_attached);
}

void CANManager::onMsgReceived()
{
    can_handle.read(rx_msg);
    for (int i = 0; i < motor_handlers_vec_size; i++) {
        motor_handlers_vec_ptr[i]->unpack(rx_msg);
    }
}

void CANManager::sendMsg()
{
    for (int i = 0; i < motor_handlers_vec_size; i++) {
        can_handle.write(motor_handlers_vec_ptr[i]->tx_msg);
    }
}
