#include "capstone.h"

#if USE_PID
static bool             pidInit(void);
static bool             pidCompute(void);
static bool             pidControl_p(void);
#endif

static void             onMsgReceived1(void);
static void             onMsgReceived2(void);

static void             write_txmsg(void);
static void             halt(void);
static bool             loadRefTbl(bool until);
static void             observe(void);
static void             debug_txmsg(void);

static void             jump(void);
static void             jump1(void);
static void             standUp(void);
static Motor::SetData   sitDown_calc(int count_down, const Motor::SetData &datum);

static void             serial_isr(void);

static void             interact(void);
static void             delta(const char *msg);

Timer                   timer;
CANMessage              tx_msg[NumberOfMotors], rx_msg;
Serial                  pc(PA_2, PA_3);
CAN                     can1(PB_8, PB_9), can2(PB_5, PB_6);
Ticker                  send_can;
IO                      io;
Mode                    mode = SetzeroMode;
long int                turn_cnt = -2;
Motor                   motors[NumberOfMotors];
float                   p_ctrls[NumberOfMotors];
void                    (*operation)(void) = standUp;
const int               count_down_CNT = -100;

/// SET ME!!!
static PIDController pids[] = {
    //            Kp    Ki    Kd     y(t)                          u(t)         r(t)
    PIDController(1.30, 0.10, 0.00, &motors[0].data_from_motor.p, &p_ctrls[0], &motors[0].data_to_motor.p, P_MIN, P_MAX),
    PIDController(1.25, 0.30, 0.00, &motors[1].data_from_motor.p, &p_ctrls[1], &motors[1].data_to_motor.p, P_MIN, P_MAX),
    PIDController(2.00, 1.00, 0.00, &motors[2].data_from_motor.p, &p_ctrls[2], &motors[2].data_to_motor.p, P_MIN, P_MAX),
    PIDController(1.30, 0.10, 0.00, &motors[3].data_from_motor.p, &p_ctrls[3], &motors[3].data_to_motor.p, P_MIN, P_MAX),
    PIDController(1.25, 0.30, 0.00, &motors[4].data_from_motor.p, &p_ctrls[4], &motors[4].data_to_motor.p, P_MIN, P_MAX),
    PIDController(2.00, 1.00, 0.00, &motors[5].data_from_motor.p, &p_ctrls[5], &motors[5].data_to_motor.p, P_MIN, P_MAX),
};

static const int CAN_FREQUENCY = 1000000;
static const int SERIAL_HZ     = 921600;

static const unsigned int CAN_ID = 0x01;
static const unsigned int ID     = CAN_ID << 21;
static const unsigned int MASK   = 0xFFE00004;

int main()
{
    send_can.attach(serial_isr, Tick_dt);

    pc.baud(SERIAL_HZ);
    pc.attach(interact);

    can1.frequency(CAN_FREQUENCY);
    can1.attach(onMsgReceived1);
    can1.filter(ID, MASK, CANStandard, 0);
    can2.frequency(CAN_FREQUENCY);
    can2.attach(onMsgReceived2);
    can2.filter(ID, MASK, CANStandard, 0);

    rx_msg.len = 6;
    for (int i = 0; i < len(tx_msg); i++) {
        tx_msg[i].len = 8;
        tx_msg[i].id  = i + 1;
    }
    for (int i = 0; i < len(motors); i++) {
        motors[i].motor_id = i + 1;
    }
    for (int i = 0; i < len(motors); i++) {
        const Motor::SetData init_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };
        motors[i].data_to_motor = init_data;
    }
    write_txmsg();

    printf("\n");
    printf("\r<< %s >>\n", CAPSTONE);
    printf("\rVERSION = %s\n", VERSION);
#if USE_PID
    printf("\rUSE_PID = true\n");
#else
    printf("\rUSE_PID = false\n");
#endif
    printf("\rREF_TBL_ID = %d\n", REF_TBL_ID);
    printf("\rRUNTIME_TICK_MAX = %d\n", RUNTIME_TICK_MAX);
    printf("\rTick_dt = %lf\n", Tick_dt);
    printf("\n");

    turn_cnt = -2;
    io.set_delta(delta);
    timer.start();
}

#if USE_PID
bool pidInit()
{
    bool res = true;
    for (int i = 0; i < len(pids); i++) {
        res &= pids[i].init();
    }
    return res;
}

bool pidCompute()
{
    bool res = true;
    for (int i = 0; i < len(pids); i++) {
        res &= pids[i].compute();
    }
    return res;
}

bool pidControl_p()
{
    bool res = true;
    res = pidCompute();
    for (int i = 0; i < len(motors); i++) {
        motors[i].data_to_motor.p = p_ctrls[i];
    }
    return res;
}
#endif

void onMsgReceived1()
{
    can1.read(rx_msg);
    for (int i = 0; i < 3; i++) {
        motors[i].unpack(&rx_msg);
    }
}

void onMsgReceived2()
{
    can2.read(rx_msg);
    for (int i = 3; i < 6; i++) {
        motors[i].unpack(&rx_msg);
    }
}

bool loadRefTbl(bool until)
{
    static Motor::SetData last_data[len(motors)] = {
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
    };

    until &= turn_cnt < len(ref_tbl);

    if (until) {
        for (int i = 0; i < len(ref_tbl[turn_cnt]); i++) {
            motors[i].data_to_motor = ref_tbl[turn_cnt][i % 3];
            last_data[i] = ref_tbl[turn_cnt][i % 3];
        }
        return true;
    }
    else {
        for (int i = 0; i < len(ref_tbl[turn_cnt]); i++) {
            motors[i].data_to_motor = last_data[i];
        }
        return false;
    }
}

void write_txmsg()
{
    for (int i = 0; i < 3; i++) {
        can1.write(tx_msg[i]);
    }
    for (int i = 3; i < 6; i++) {
        can2.write(tx_msg[i]);
    }
}

void halt()
{
    const Motor::SetData zero_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };

    for (int i = 0; i < len(motors); i++) {
        motors[i].data_to_motor = zero_data;
    }
    mode = SetzeroMode;
    turn_cnt = -2;
}

void observe()
{
    static long int row = 0;
    static Gear gear_obs = Gear(20);

    if (gear_obs.go()) {
        if (turn_cnt >= 0) {
            row++;
        }
        else {
            row = 0;
        }
#if 0
        printf("\r%% turn_cnt = %d\n", turn_cnt);
#endif
        for (int i = 0; i < len(motors); i++) {
            const Motor::SetData data = motors[i].data_to_motor;
            printf("\rtheta(%ld, %d) = %f; omega(%ld, %d) = %f;\n", row, motors[i].motor_id, data.p, row, motors[i].motor_id, data.v);
        }
        printf("\n");
    }
}

void debug_txmsg()
{
    static Gear gear_ovw = Gear(20);

    if (gear_ovw.go()) {
        for (int i = 0; i < len(tx_msg); i++) {
            const Motor::SetData ovw = decode16(&tx_msg[i].data);
            printf("\n\r%% tx_msg[%d] = { .p = %.4lf, .v = %.4lf, .kp = %.4lf, .kd = %.4lf, .t_ff = %.4lf }\n", i, ovw.p, ovw.v, ovw.kp, ovw.kd, ovw.t_ff);
        }
        printf("\n");
    }
}

void jump()
{
    loadRefTbl(turn_cnt < len(ref_tbl));
}

void jump1()
{
    loadRefTbl(turn_cnt <= PID_START_TICK);

    if (turn_cnt == PID_START_TICK) {
        pidInit();
    }
    else if (turn_cnt > PID_START_TICK) {
        pidControl_p();
    }
}

void standUp()
{
    const unsigned char lines[3][8] = {
        { 0x7C, 0xA5, 0x96, 0xB0, 0x7A, 0x99, 0x97, 0xFF, },
        { 0x7C, 0xED, 0x96, 0xB0, 0x7A, 0x99, 0x95, 0xC6, },
        { 0x7F, 0xBA, 0x7F, 0xF0, 0x39, 0x00, 0x07, 0x8D, },
    };

    for (int i = 0; i < len(motors); i++) {
        motors[i].data_to_motor = decode16(&lines[i % 3]);
    }
}

Motor::SetData sitDown_calc(const int count_down, const Motor::SetData &datum)
{
    const Motor::SetData res = {
        .p    = abs(count_down) * datum.p / abs(count_down_CNT),
        .v    = abs(count_down) * datum.v / abs(count_down_CNT),
        .kp   = abs(count_down) * datum.kp / abs(count_down_CNT),
        .kd   = abs(count_down) * datum.kd / abs(count_down_CNT),
        .t_ff = abs(count_down) * datum.t_ff / abs(count_down_CNT),
    };

    return res;
}

void serial_isr()
{
    switch (mode) {
    case RuntimeMode:
        if (turn_cnt > RUNTIME_TICK_MAX) {
            halt();
        }
        else if (turn_cnt >= 0) {
            operation();
            observe();
            turn_cnt++;
        }
        for (int i = 0; i < len(motors); i++) {
            motors[i].pack(&tx_msg[i]);
        }
        break;
    case ObserveMode:
        turn_cnt = -2;
        observe();
        break;
    case ReadcmdMode:
        turn_cnt = -2;
        break;
    case SetzeroMode:
        turn_cnt = -2;
        halt();
        break;
    case SitdownMode:
        if (turn_cnt >= -2) {
            turn_cnt = -2;
            halt();
        }
        else if (turn_cnt >= count_down_CNT) {
            observe();
            for (int i = 0; i < len(motors); i++) {
                const Motor::SetData datum = sitDown_calc(-turn_cnt, motors[i].data_to_motor);
                motors[i].pack(&tx_msg[i]);
            }
            turn_cnt++;
        }
        break;
    default:
        printf("\n\r%% Undefined mode %%\n");
        mode = SetzeroMode;
        turn_cnt = -2;
        halt();
        break;
    }

#if 0
    debug_txmsg();
#endif
    write_txmsg();
}

void interact()
{
    char ch = '\0';
#if USE_PID
    bool pid_okay = true;
#endif

    if (mode == ReadcmdMode) {
        bool delta_routine_breaked = io.run_delta();
        if (delta_routine_breaked) {
            mode = SetzeroMode;
        }
        return;
    }
    if ((ch = IO::getc()) == 0) {
        return;
    }
    else {
        switch (ch) {
        case ESC:
            printf("\n\r%% Exiting motor mode %%\n");
            for (int i = 0; i < len(tx_msg); i++) {
                tx_msg[i].data[0] = 0xFF;
                tx_msg[i].data[1] = 0xFF;
                tx_msg[i].data[2] = 0xFF;
                tx_msg[i].data[3] = 0xFF;
                tx_msg[i].data[4] = 0xFF;
                tx_msg[i].data[5] = 0xFF;
                tx_msg[i].data[6] = 0xFF;
                tx_msg[i].data[7] = 0xFD;
            }
            turn_cnt = -1;
            break;
        case 'm':
            printf("\n\r%% Entering motor mode %%\n");
            for (int i = 0; i < len(tx_msg); i++) {
                tx_msg[i].data[0] = 0xFF;
                tx_msg[i].data[1] = 0xFF;
                tx_msg[i].data[2] = 0xFF;
                tx_msg[i].data[3] = 0xFF;
                tx_msg[i].data[4] = 0xFF;
                tx_msg[i].data[5] = 0xFF;
                tx_msg[i].data[6] = 0xFF;
                tx_msg[i].data[7] = 0xFC;
            }
            turn_cnt = -1;
            break;
        case 'z':
            printf("\n\r%% Set zero %%\n");
            for (int i = 0; i < len(tx_msg); i++) {
                tx_msg[i].data[0] = 0xFF;
                tx_msg[i].data[1] = 0xFF;
                tx_msg[i].data[2] = 0xFF;
                tx_msg[i].data[3] = 0xFF;
                tx_msg[i].data[4] = 0xFF;
                tx_msg[i].data[5] = 0xFF;
                tx_msg[i].data[6] = 0xFF;
                tx_msg[i].data[7] = 0xFE;
            }
            break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
            for (int j = 0; j < 6; j++) {
                if (j < len(tx_msg)) {
                    if (ch == "123456"[j]) {
                        printf("\n\r%% Motor%c rest position %%\n", ch);
                        tx_msg[j].data[0] = 0x7F;
                        tx_msg[j].data[1] = 0xFF;
                        tx_msg[j].data[2] = 0x7F;
                        tx_msg[j].data[3] = 0xF0;
                        tx_msg[j].data[4] = 0x00;
                        tx_msg[j].data[5] = 0x00;
                        tx_msg[j].data[6] = 0x07;
                        tx_msg[j].data[7] = 0xFF;
                    }
                }
            }
            break;
        case 'r':
            printf("\n\r%% Run %%\n");
            mode = RuntimeMode;
            turn_cnt = 0;
#if USE_PID
            pid_okay &= pidInit();
            if (!pid_okay) {
                printf("\n\r%% Initializing PID failed %%\n");
                mode = SetzeroMode;
                turn_cnt = -2;
            }
#endif
            return;
        case 'o':
            printf("\n\r%% Observe %%\n");
            mode = ObserveMode;
            turn_cnt = -2;
            return;
        case 'b':
            printf("\n\r%% Break %%\n");
            mode = SetzeroMode;
            turn_cnt = -2;
            halt();
            return;
        case 'l':
            if (mode == SetzeroMode && turn_cnt < 0) {
                printf("\n\r%% Listen %%\n");
                mode = ReadcmdMode;
                turn_cnt = -2;
            }
            return;
        }
    }
    write_txmsg();
}

void delta (const char *const msg)
{   
    int sscanf_res = 0;
    char var_name[16];
    char op_name[16];
    double value;
    int motor_id = 0;
    bool res = false;

    if (msg == NULL) {
        printf("\n\r%% Leaving listening mode %%\n");
        mode = SetzeroMode;
        return;
    }

#if USE_PID
    sscanf_res = sscanf(msg, "%s %d = %lf", var_name, &motor_id, &value);
    if (sscanf_res == 3) {
        if (areSameStr("Kp", var_name)) {
            pids[motor_id - 1].Kp = value;
            res = true;
            return;
        }
        else if (areSameStr("Ki", var_name)) {
            pids[motor_id - 1].Ki = value;
            res = true;
            return;
        }
        else if (areSameStr("Kd", var_name)) {
            pids[motor_id - 1].Kd = value;
            res = true;
            return;
        }
        else {
            res = false;
        }
        goto RET;
    }
#endif

    sscanf_res = scanf(msg, "%s", op_name);
    if (sscanf_res == 1) {
        if (areSameStr(op_name, "jump")) {
            operation = jump;
            res = true;
        }
        else if (areSameStr(op_name, "jump1")) {
            operation = jump1;
            res = true;
        }
        else if (areSameStr(op_name, "standup")) {
            operation = standUp;
            res = true;
        }
        else {
            res = false;
        }
        goto RET;
    }

RET:
    if (res) {
        printf("\n\rCommand well recieved\n");
    }
    else {
        printf("\n\rUnknown command or wrong command\n");
    }
}
