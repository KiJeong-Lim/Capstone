#include "capstone.h"

int                 turn_cnt  = -2;
const char          *io_input = NULL;

Timer               timer;
CANMessage          rx_msg, tx_msg[NumberOfMotors];
Serial              pc(PA_2, PA_3);
CAN                 can(PB_8, PB_9);
Ticker              send_can;

MotorInput          mtr_input[NumberOfMotors];
MotorOutput         mtr_output[NumberOfMotors];

static const int    CAN_FREQUENCY = 1000000;
static const int    SERIAL_HZ     = 921600;

static const unsigned int CAN_ID = 0x01;
static const unsigned int ID     = CAN_ID << 21;
static const unsigned int MASK   = 0xFFE00004;

int main()
{
    send_can.attach(serialIsr, tick_dt);
    pc.baud(SERIAL_HZ);
    pc.attach(interact);
    can.frequency(CAN_FREQUENCY);
    can.attach(onMsgReceived);
    can.filter(ID, MASK, CANStandard, 0);

    turn_cnt = -2;
    io_input = NULL;
    rx_msg.len = 6;
    for (int i = 0; i < len(tx_msg); i++) {
        tx_msg[i].len = 8;
        tx_msg[i].id  = i + 1;
    }
    for (int i = 0; i < len(mtr_output); i++) {
        mtr_output[i].set_id(i + 1);
    }
    for (int i = 0; i < len(mtr_input); i++) {
        const MotorInputData init_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };
        mtr_input[i].init();
        mtr_input[i].set(init_data);
        mtr_input[i].pack(&tx_msg[i]);
    }
    printf("\n");
    printf("VERSION    = %s\n", VERSION_H);
#if USE_PID
    printf("USE_PID    = true\n");
#else
    printf("USE_PID    = false\n");
#endif
    printf("REF_TBL_ID = %d\n", REF_TBL_ID);
    printf("RUNTIME_TICK_CNT_MAX = %d\n", RUNTIME_TICK_CNT_MAX);
    printf("tick_dt = %lf\n", tick_dt);
    printf("\n");

    for (int i = 0; i < len(tx_msg); i++) {
        can.write(tx_msg[i]);
    }
    timer.start();
}
