#include "capstone.h"

Serial      pc(PA_2, PA_3);
CAN         can(PB_8, PB_9);
Timer       timer;
Ticker      sendCAN;
int         turn_cnt = 0;

CANMessage  rxMsg;
CANMessage  txMsg1;
CANMessage  txMsg2;
CANMessage  txMsg3;

int main(void)
{
    timer.start();
    turn_cnt = -1;
    sendCAN.attach(&serial_isr, dt);
    pc.baud(921600); pc.attach(&command);
    can.frequency(1000000); can.attach(&onMsgReceived); can.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);
    txMsg1.len = 8; txMsg2.len = 8; txMsg3.len = 8; rxMsg.len = 6;
    txMsg1.id = 1; txMsg2.id = 2; txMsg3.id = 3;
    pack_cmd(txMsg1, 0, 0, 0, 0, 0); pack_cmd(txMsg2, 0, 0, 0, 0, 0); pack_cmd(txMsg3, 0, 0, 0, 0, 0);
}
