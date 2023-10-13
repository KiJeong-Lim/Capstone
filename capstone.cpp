#include "capstone.h"

static float theta1, theta2, theta3, dtheta1, dtheta2, dtheta3;
static float p1_ref, v1_ref, kp1_ref, kd1_ref, t1_ref;
static float p2_ref, v2_ref, kp2_ref, kd2_ref, t2_ref;
static float p3_ref, v3_ref, kp3_ref, kd3_ref, t3_ref;

static enum Mode mode = setzero_mode;

void serial_isr(void)
{
    p1_ref = refs_tbl[turn_cnt][0].p_ref; v1_ref = refs_tbl[turn_cnt][0].v_ref; kp1_ref = refs_tbl[turn_cnt][0].kp_ref; kd1_ref = refs_tbl[turn_cnt][0].kd_ref; t1_ref = refs_tbl[turn_cnt][0].t_ref;
    p2_ref = refs_tbl[turn_cnt][1].p_ref; v2_ref = refs_tbl[turn_cnt][1].v_ref; kp2_ref = refs_tbl[turn_cnt][1].kp_ref; kd2_ref = refs_tbl[turn_cnt][1].kd_ref; t2_ref = refs_tbl[turn_cnt][1].t_ref;
    p3_ref = refs_tbl[turn_cnt][2].p_ref; v3_ref = refs_tbl[turn_cnt][2].v_ref; kp3_ref = refs_tbl[turn_cnt][2].kp_ref; kd3_ref = refs_tbl[turn_cnt][2].kd_ref; t3_ref = refs_tbl[turn_cnt][2].t_ref;
    switch (mode) {
    case runtime_mode:
        if (turn_cnt > 1000) {
            pack_cmd(txMsg1, 0.0, 0.0, 0.0, 0.0, 0.0);
            pack_cmd(txMsg2, 0.0, 0.0, 0.0, 0.0, 0.0);
            pack_cmd(txMsg3, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
        else if (turn_cnt >= 0) {
            pack_cmd(txMsg1, p1_ref, v1_ref, kp1_ref, kd1_ref, t1_ref);
            pack_cmd(txMsg2, p2_ref, v2_ref, kp2_ref, kd2_ref, t2_ref);
            pack_cmd(txMsg3, p3_ref, v3_ref, kp3_ref, kd3_ref, t3_ref);
            printf("theta1: %lf, omega1: %lf\n", theta1, dtheta1);
            printf("theta2: %lf, omega2: %lf\n", theta2, dtheta2);
            printf("theta3: %lf, omega3: %lf\n", theta3, dtheta3);
            printf("\n");
            turn_cnt++;
        }
        break;
    case observe_mode:
        printf("theta1: %lf, omega1: %lf\n", theta1, dtheta1);
        printf("theta2: %lf, omega2: %lf\n", theta2, dtheta2);
        printf("theta3: %lf, omega3: %lf\n", theta3, dtheta3);
        printf("\n");
        turn_cnt = -1;
        break;
    case setzero_mode:
    default:
        pack_cmd(txMsg1, 0.0, 0.0, 0.0, 0.0, 0.0);
        pack_cmd(txMsg2, 0.0, 0.0, 0.0, 0.0, 0.0);
        pack_cmd(txMsg3, 0.0, 0.0, 0.0, 0.0, 0.0);
        turn_cnt = -1;
    }
    can.write(txMsg1); can.write(txMsg2); can.write(txMsg3);
}

void command(void)
{
    char ch = '\0';
    while (pc.readable()) {
        ch = pc.getc();
        switch (ch) {
        case 27:
            printf("\n\r Exiting motor mode \n\r");
            txMsg1.data[0] = 0xFF; txMsg1.data[1] = 0xFF; txMsg1.data[2] = 0xFF; txMsg1.data[3] = 0xFF; txMsg1.data[4] = 0xFF; txMsg1.data[5] = 0xFF; txMsg1.data[6] = 0xFF; txMsg1.data[7] = 0xFD;
            txMsg2.data[0] = 0xFF; txMsg2.data[1] = 0xFF; txMsg2.data[2] = 0xFF; txMsg2.data[3] = 0xFF; txMsg2.data[4] = 0xFF; txMsg2.data[5] = 0xFF; txMsg2.data[6] = 0xFF; txMsg2.data[7] = 0xFD;
            txMsg3.data[0] = 0xFF; txMsg3.data[1] = 0xFF; txMsg3.data[2] = 0xFF; txMsg3.data[3] = 0xFF; txMsg3.data[4] = 0xFF; txMsg3.data[5] = 0xFF; txMsg3.data[6] = 0xFF; txMsg3.data[7] = 0xFD;
            break;
        case 'm':
            printf("\n\r Entering motor mode \n\r");
            txMsg1.data[0] = 0xFF; txMsg1.data[1] = 0xFF; txMsg1.data[2] = 0xFF; txMsg1.data[3] = 0xFF; txMsg1.data[4] = 0xFF; txMsg1.data[5] = 0xFF; txMsg1.data[6] = 0xFF; txMsg1.data[7] = 0xFC;
            txMsg2.data[0] = 0xFF; txMsg2.data[1] = 0xFF; txMsg2.data[2] = 0xFF; txMsg2.data[3] = 0xFF; txMsg2.data[4] = 0xFF; txMsg2.data[5] = 0xFF; txMsg2.data[6] = 0xFF; txMsg2.data[7] = 0xFC;
            txMsg3.data[0] = 0xFF; txMsg3.data[1] = 0xFF; txMsg3.data[2] = 0xFF; txMsg3.data[3] = 0xFF; txMsg3.data[4] = 0xFF; txMsg3.data[5] = 0xFF; txMsg3.data[6] = 0xFF; txMsg3.data[7] = 0xFC;
            break;
        case 'z':
            printf("\n\r Set zero \n\r");
            txMsg1.data[0] = 0xFF; txMsg1.data[1] = 0xFF; txMsg1.data[2] = 0xFF; txMsg1.data[3] = 0xFF; txMsg1.data[4] = 0xFF; txMsg1.data[5] = 0xFF; txMsg1.data[6] = 0xFF; txMsg1.data[7] = 0xFE;
            txMsg2.data[0] = 0xFF; txMsg2.data[1] = 0xFF; txMsg2.data[2] = 0xFF; txMsg2.data[3] = 0xFF; txMsg2.data[4] = 0xFF; txMsg2.data[5] = 0xFF; txMsg2.data[6] = 0xFF; txMsg2.data[7] = 0xFE;
            txMsg3.data[0] = 0xFF; txMsg3.data[1] = 0xFF; txMsg3.data[2] = 0xFF; txMsg3.data[3] = 0xFF; txMsg3.data[4] = 0xFF; txMsg3.data[5] = 0xFF; txMsg3.data[6] = 0xFF; txMsg3.data[7] = 0xFE;
            break;
        case '1':
            printf("\n\r 1st motor rest position \n\r");
            txMsg1.data[0] = 0x7F; txMsg1.data[1] = 0xFF; txMsg1.data[2] = 0x7F; txMsg1.data[3] = 0xf0; txMsg1.data[4] = 0x00; txMsg1.data[5] = 0x00; txMsg1.data[6] = 0x07; txMsg1.data[7] = 0xFF;
            break;
        case '2':
            printf("\n\r 2nd motor rest position \n\r");
            txMsg2.data[0] = 0x7F; txMsg2.data[1] = 0xFF; txMsg2.data[2] = 0x7F; txMsg2.data[3] = 0xF0; txMsg2.data[4] = 0x00; txMsg2.data[5] = 0x00; txMsg2.data[6] = 0x07; txMsg2.data[7] = 0xFF;               
            break;
        case '3':
            printf("\n\r 3rd motor rest position \n\r");
            txMsg3.data[0] = 0x7F; txMsg3.data[1] = 0xFF; txMsg3.data[2] = 0x7F; txMsg3.data[3] = 0xF0; txMsg3.data[4] = 0x00; txMsg3.data[5] = 0x00; txMsg3.data[6] = 0x07; txMsg3.data[7] = 0xFF;
            break;
        case 'g':
            mode = runtime_mode;
            turn_cnt = 0;
#if 0
        case 'a':
            printf("\n\r sit down \n\r");
            txMsg1.data[0] = 0x7F; txMsg1.data[1] = 0xA9; txMsg1.data[2] = 0x7F; txMsg1.data[3] = 0xF0; txMsg1.data[4] = 0x2A; txMsg1.data[5] = 0x66; txMsg1.data[6] = 0x67; txMsg1.data[7] = 0x82;
            txMsg2.data[0] = 0x7F; txMsg2.data[1] = 0x65; txMsg2.data[2] = 0x7F; txMsg2.data[3] = 0xF0; txMsg2.data[4] = 0x28; txMsg2.data[5] = 0x66; txMsg2.data[6] = 0x66; txMsg2.data[7] = 0x38;
            txMsg3.data[0] = 0x7F; txMsg3.data[1] = 0xFF; txMsg3.data[2] = 0x7F; txMsg3.data[3] = 0xF0; txMsg3.data[4] = 0x51; txMsg3.data[5] = 0x4C; txMsg3.data[6] = 0xC7; txMsg3.data[7] = 0xFF;
            break;
        case 'q':
            printf("\n\r Test \n\r");
            pack_cmd(txMsg1, 0, 0, 0, 0, 0);
            pack_cmd(txMsg2, 0, 0, 0, 0, 0);
            pack_cmd(txMsg3, -0.20, 0, 4, 3, 0);
            break;
        case 'w':
            printf("\n\r Test \n\r");
            pack_cmd(txMsg1, -0.1, 0, 18, 3.5, 0);
            pack_cmd(txMsg2, -0.115, 0, 18, 3.5, 0);
            pack_cmd(txMsg3, 0, 0, 15, 3, 0);
            break;
#endif
        case 'o':
            printf("\n\r Observe \n\r");
            mode = observe_mode;
            turn_cnt = -1;
            break;
        case 'b':
            mode = setzero_mode;
            turn_cnt = -1;
            break;
        }
    }
    can.write(txMsg1); can.write(txMsg2); can.write(txMsg3);
}

void pack_cmd(CANMessage &msg, float p_des, float v_des, float kp, float kd, float t_ff)
{
    // limit data to be within bounds //
    p_des = min(max(P_MIN, p_des), P_MAX);                    
    v_des = min(max(V_MIN, v_des), V_MAX);
    kp = min(max(KP_MIN, kp), KP_MAX);
    kd = min(max(KD_MIN, kd), KD_MAX);
    t_ff = min(max(T_MIN, t_ff), T_MAX);
    // convert floats to unsigned ints //
    int const p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
    int const v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
    int const kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    int const kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
     int const t_int  = float_to_uint(t_ff, T_MIN, T_MAX, 12);
    // pack ints into the can buffer //
    msg.data[0] = p_int>>8; msg.data[1] = p_int&0xFF; msg.data[2] = v_int>>4; msg.data[3] = ((v_int&0xF)<<4)|(kp_int>>8); msg.data[4] = kp_int&0xFF; msg.data[5] = kd_int>>4; msg.data[6] = ((kd_int&0xF)<<4)|(t_int>>8); msg.data[7] = t_int&0xff;
}

void unpack_reply(CANMessage msg)
{
    // unpack ints from can buffer //
    int const id = msg.data[0];
    int const p_int = (msg.data[1]<<8)|msg.data[2];
    int const v_int = (msg.data[3]<<4)|(msg.data[4]>>4);
    int const i_int = ((msg.data[4]&0xF)<<8)|msg.data[5];
    // convert ints to floats //
    float const p = uint_to_float(p_int, P_MIN, P_MAX, 16);
    float const v = uint_to_float(v_int, V_MIN, V_MAX, 12);
    float const i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
    if (id == 1) {
        theta1 = p;
        dtheta1 = v;
    }
    else if (id == 2) {
        theta2 = p;
        dtheta2 = v;
    }
    else if (id == 3) {
        theta3 = p;
        dtheta3 = v;
    }
}

void onMsgReceived(void)
{
    can.read(rxMsg);
    unpack_reply(rxMsg);
}
