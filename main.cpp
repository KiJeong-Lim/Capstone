#include "capstone.h"

Serial      pc(PA_2, PA_3);
CAN         can(PB_8, PB_9);
Timer       timer;
Ticker      sendCAN;
int         turn_cnt = 0;
bool        live = true;

CANMessage  rxMsg;
CANMessage  txMsg1;
CANMessage  txMsg2;
CANMessage  txMsg3;

int main(void)
{
    timer.start();
    live = true;
    turn_cnt = 0;

    sendCAN.attach(&serial_isr, dt);

    pc.baud(921600);
    pc.attach(&command);

    can.frequency(1000000);
    can.attach(&onMsgReceived);
    can.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);

    txMsg1.len = 8;                         //transmit 8 bytes
    txMsg2.len = 8;                         //transmit 8 bytes
    txMsg3.len = 8;                         //transmit 8 bytes
    rxMsg.len = 6;                          //receive 5 bytes

    txMsg1.id = 1;                        //1st motor ID
    txMsg2.id = 2;                        //2nd motor ID
    txMsg3.id = 3;                        //3rd motor ID

    pack_cmd(txMsg1, 0, 0, 0, 0, 0);       //Start out by sending all 0's
    pack_cmd(txMsg2, 0, 0, 0, 0, 0);
    pack_cmd(txMsg3, 0, 0, 0, 0, 0);
}
