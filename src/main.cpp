#include "capstone.h"

class MotorHandler : public Motor {
public:
    CANMessage tx_msg;
public:
    MotorHandler(const int id)
    {
        tx_msg.len = 8;
        tx_msg.id = id;
        this->motor_id = id;
    }
    void put_txmsg(UCh8 rhs)
    {
        for (int i = 0; i < 8; i++) {
            tx_msg.data[i] = rhs.data[i];
        }
    }
};

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
static void             prompt(const char *msg);

CANManager      cans[] = { CANManager(PB_8, PB_9), CANManager(PB_5, PB_6) };
void            (*const onMsgReceived[])(void) = { onMsgReceived1, onMsgReceived2 };
MotorHandler    motor_handlers[] = { MotorHandler(1), MotorHandler(2), MotorHandler(3), MotorHandler(4), MotorHandler(5), MotorHandler(6), }; // SET ME !!!
IO              terminal;
Timer           timer;
Ticker          send_can;
CANMessage      rx_msg;
float           p_ctrls[len(motor_handlers)];
Serial          pc(PA_2, PA_3);

Mode        mode                = SetzeroMode;
long int    turn_cnt            = -2;
void        (*operation)(void)  = standUp;
const int   count_down_MAX_CNT  = -100;

static PIDController pids[] = {
    //            Kp    Ki    Kd     y(t)                                  u(t)         r(t)
    PIDController(1.30, 0.10, 0.00, &motor_handlers[0].data_from_motor.p, &p_ctrls[0], &motor_handlers[0].data_to_motor.p, P_MIN, P_MAX), // SET ME !!!
    PIDController(1.25, 0.30, 0.00, &motor_handlers[1].data_from_motor.p, &p_ctrls[1], &motor_handlers[1].data_to_motor.p, P_MIN, P_MAX), // SET ME !!!
    PIDController(2.00, 1.00, 0.00, &motor_handlers[2].data_from_motor.p, &p_ctrls[2], &motor_handlers[2].data_to_motor.p, P_MIN, P_MAX), // SET ME !!!
    PIDController(1.30, 0.10, 0.00, &motor_handlers[3].data_from_motor.p, &p_ctrls[3], &motor_handlers[3].data_to_motor.p, P_MIN, P_MAX), // SET ME !!!
    PIDController(1.25, 0.30, 0.00, &motor_handlers[4].data_from_motor.p, &p_ctrls[4], &motor_handlers[4].data_to_motor.p, P_MIN, P_MAX), // SET ME !!!
    PIDController(2.00, 1.00, 0.00, &motor_handlers[5].data_from_motor.p, &p_ctrls[5], &motor_handlers[5].data_to_motor.p, P_MIN, P_MAX), // SET ME !!!
};

int main()
{
    send_can.attach(serial_isr, Tick_dt);

    pc.baud(921600);
    pc.attach(interact);

    rx_msg.len = 6;
    for (int i = 0; i < len(cans); i++) {
        cans[i].init(0x01 << 21, 0xFFE00004, onMsgReceived[i]);
    }

    for (int i = 0; i < len(motor_handlers); i++) {
        const Motor::SetData init_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };
        motor_handlers[i].data_to_motor = init_data;
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
    terminal.set_prompt(prompt);
    timer.start();
}

void onMsgReceived1()
{
    cans[0].get().read(rx_msg);
    motor_handlers[0].unpack(&rx_msg); // SET ME !!!
    motor_handlers[1].unpack(&rx_msg); // SET ME !!!
    motor_handlers[2].unpack(&rx_msg); // SET ME !!!
}

void onMsgReceived2()
{
    cans[1].get().read(rx_msg);
    motor_handlers[3].unpack(&rx_msg); // SET ME !!!
    motor_handlers[4].unpack(&rx_msg); // SET ME !!!
    motor_handlers[5].unpack(&rx_msg); // SET ME !!!
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

bool loadRefTbl(bool until)
{
    static Motor::SetData last_data[len(motor_handlers)] = {
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
            motor_handlers[i].data_to_motor = ref_tbl[turn_cnt][i % 3];
            last_data[i] = ref_tbl[turn_cnt][i % 3];
        }
        return true;
    }
    else {
        for (int i = 0; i < len(ref_tbl[turn_cnt]); i++) {
            motor_handlers[i].data_to_motor = last_data[i];
        }
        return false;
    }
}

void write_txmsg()
{
    for (int i = 0; i < len(motor_handlers); i++) {
        cans[i / 3].get().write(motor_handlers[i].tx_msg);
    }
}

void halt()
{
    const Motor::SetData zero_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };

    for (int i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].data_to_motor = zero_data;
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
        for (int i = 0; i < len(motor_handlers); i++) {
            const Motor::SetData data = motor_handlers[i].data_to_motor;
            printf("\rtheta(%ld, %d) = %f; omega(%ld, %d) = %f;\n", row, motor_handlers[i].motor_id, data.p, row, motor_handlers[i].motor_id, data.v);
        }
        printf("\n");
    }
}

void debug_txmsg()
{
    static Gear gear_ovw = Gear(20);

    if (gear_ovw.go()) {
        for (int i = 0; i < len(motor_handlers); i++) {
            const Motor::SetData ovw = decode16(&motor_handlers[i].tx_msg.data);
            printf("\n\r%% tx_msg[%d] = { .p = %.4lf, .v = %.4lf, .kp = %.4lf, .kd = %.4lf, .t_ff = %.4lf, }\n", i, ovw.p, ovw.v, ovw.kp, ovw.kd, ovw.t_ff);
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
#if USE_PID
    if (turn_cnt == PID_START_TICK) {
        pidInit();
    }
    else if (turn_cnt > PID_START_TICK) {
        pidControl_p();
    }
#endif
}

void standUp()
{
    const unsigned char lines[3][8] = {
        { 0x7C, 0xA5, 0x96, 0xB0, 0x7A, 0x99, 0x97, 0xFF, },
        { 0x7C, 0xED, 0x96, 0xB0, 0x7A, 0x99, 0x95, 0xC6, },
        { 0x7F, 0xBA, 0x7F, 0xF0, 0x39, 0x00, 0x07, 0x8D, },
    };

    for (int i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].data_to_motor = decode16(&lines[i % 3]);
    }
}

Motor::SetData sitDown_calc(const int count_down, const Motor::SetData &datum)
{
    const Motor::SetData res = {
        .p    = (datum.p    * abs(count_down)) / abs(count_down_MAX_CNT),
        .v    = (datum.v    * abs(count_down)) / abs(count_down_MAX_CNT),
        .kp   = (datum.kp   * abs(count_down)) / abs(count_down_MAX_CNT),
        .kd   = (datum.kd   * abs(count_down)) / abs(count_down_MAX_CNT),
        .t_ff = (datum.t_ff * abs(count_down)) / abs(count_down_MAX_CNT),
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
        for (int i = 0; i < len(motor_handlers); i++) {
            motor_handlers[i].pack(&motor_handlers[i].tx_msg);
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
        else if (turn_cnt >= count_down_MAX_CNT) {
            observe();
            for (int i = 0; i < len(motor_handlers); i++) {
                const Motor::SetData datum = sitDown_calc(-turn_cnt, motor_handlers[i].data_to_motor);
                motor_handlers[i].pack(&motor_handlers[i].tx_msg);
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

#if DEBUG_TXMSG
    debug_txmsg();
#endif
    write_txmsg();
}

void interact()
{
    int ch = '\0';
#if USE_PID
    bool pid_okay = true;
#endif

    if (mode == ReadcmdMode) {
        const bool prompt_routine_breaked = terminal.run_prompt();
        if (prompt_routine_breaked) {
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
            for (int i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, } };
                motor_handlers[i].put_txmsg(msg);
            }
            turn_cnt = -1;
            break;
        case 'm':
            printf("\n\r%% Entering motor mode %%\n");
            for (int i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, } };
                motor_handlers[i].put_txmsg(msg);
            }
            turn_cnt = -1;
            break;
        case 'z':
            printf("\n\r%% Set zero %%\n");
            for (int i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, } };
                motor_handlers[i].put_txmsg(msg);
            }
            break;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
            for (int i = 0; i < len(motor_handlers); i++) {
                if (ch == "0123456789"[motor_handlers[i].motor_id]) {
                    const UCh8 msg = { .data = { 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF, } };
                    printf("\n\r%% Motor #%c rest position %%\n", ch);
                    motor_handlers[i].put_txmsg(msg);
                    break;
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

void prompt (const char *const msg)
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
