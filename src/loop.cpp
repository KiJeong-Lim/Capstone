#include "capstone.h"

#if USE_PID
static bool             pidInit(void);
static bool             pidCompute(void);
static bool             pidControl_p(void);
#endif

static bool             loadRefTbl(bool until);
static void             halt(void);
static void             observe(void);
static void             overwatch(void);

static void             jump(void);
static void             jump1(void);
static void             standUp(void);
static MotorInputData   sitDown(int count_down, const MotorInputData &datum);

static int              console(void);

static Mode_t       mode               = SetzeroMode;
static const int    count_down_CNT     = -100;
static void         (*operation)(void) = &jump1;

static float        p_ctrls[NumberOfMotors];

/// SET ME!!!
static PIDController pids[] = {
    //            Kp    Ki    Kd     y(t)              u(t)         r(t)
    PIDController(1.30, 0.10, 0.00, &mtr_output[0].p, &p_ctrls[0], &mtr_input[0].p, P_MIN, P_MAX),
    PIDController(1.25, 0.30, 0.00, &mtr_output[1].p, &p_ctrls[1], &mtr_input[1].p, P_MIN, P_MAX),
    PIDController(2.00, 1.00, 0.00, &mtr_output[2].p, &p_ctrls[2], &mtr_input[2].p, P_MIN, P_MAX),
    PIDController(1.30, 0.10, 0.00, &mtr_output[3].p, &p_ctrls[3], &mtr_input[3].p, P_MIN, P_MAX),
    PIDController(1.25, 0.30, 0.00, &mtr_output[4].p, &p_ctrls[4], &mtr_input[4].p, P_MIN, P_MAX),
    PIDController(2.00, 1.00, 0.00, &mtr_output[5].p, &p_ctrls[5], &mtr_input[5].p, P_MIN, P_MAX),
};

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
    pidCompute();
    for (int i = 0; i < len(mtr_input); i++) {
        res &= mtr_input[i].set_p(p_ctrls[i], 10);
    }
    return res;
}
#endif

bool loadRefTbl(bool until)
{
    static MotorInputData last_data[len(mtr_input)] = {
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
        { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 },
    };

    until &= turn_cnt < len(refs_tbl);

    if (until) {
        for (int i = 0; i < len(refs_tbl[turn_cnt]); i++) {
            mtr_input[i].pull(refs_tbl[turn_cnt][i % 3]);
            last_data[i] = mtr_input[i];
        }
        return true;
    }
    else {
        for (int i = 0; i < len(refs_tbl[turn_cnt]); i++) {        
            mtr_input[i].pull(last_data[i]);
        }
        return false;
    }
}

void halt()
{
    const MotorInputData zero_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };

    for (int i = 0; i < len(mtr_input); i++) {
        mtr_input[i].init();
        mtr_input[i].set(zero_data);
    }
    mode = SetzeroMode;
    turn_cnt = -2;
}

void observe()
{
    static int row = 0;
    static Gear gear_obs = Gear(20);

    if (gear_obs.go()) {
        if (turn_cnt >= 0) {
            row++;
        }
        else {
            row = 0;
        }
        for (int i = 0; i < len(mtr_output); i++) {
            const int no = i + 1;
            const MotorOutputData data = mtr_output[i].get();
            printf("theta(%d, %d) = %f; omega(%d, %d) = %f;\n", row, no, data.p, row, no, data.v);
        }
#if 0
        printf("%% turn_cnt = %d\n", turn_cnt);
#endif
        printf("\n");
    }
}

void overwatch()
{
    static Gear gear_ovw = Gear(20);

    if (gear_ovw.go()) {
        for (int i = 0; i < len(tx_msg); i++) {
            const MotorInputData ovw = decode16(&tx_msg[i].data);
            printf("%% tx_msg[%d] = { .p = %.4lf, .v = %.4lf, .kp = %.4lf, .kd = %.4lf, .t_ff = %.4lf }\n", i, ovw.p, ovw.v, ovw.kp, ovw.kd, ovw.t_ff);
        }
        printf("\n");
    }
}

void jump()
{
    loadRefTbl(turn_cnt < len(refs_tbl));
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

    for (int i = 0; i < len(mtr_input); i++) {
        mtr_input[i].hexadecimal(&lines[i % 3]);
    }
}

MotorInputData sitDown(const int count_down, const MotorInputData &datum)
{
    const MotorInputData res = {
        .p    = abs(count_down) * datum.p / abs(count_down_CNT),
        .v    = abs(count_down) * datum.v / abs(count_down_CNT),
        .kp   = abs(count_down) * datum.kp / abs(count_down_CNT),
        .kd   = abs(count_down) * datum.kd / abs(count_down_CNT),
        .t_ff = abs(count_down) * datum.t_ff / abs(count_down_CNT),
    };
    return res;
}

void serialIsr()
{
    switch (mode) {
    case RuntimeMode:
        if (turn_cnt > RUNTIME_TICK_CNT_MAX) {
            halt();
        }
        else if (turn_cnt >= 0) {
            operation();
            observe();
            turn_cnt++;
        }
        for (int i = 0; i < len(mtr_output); i++) {
            mtr_input[i].pack(&tx_msg[i]);
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
    case GotobedMode:
        if (turn_cnt >= -2) {
            turn_cnt = -2;
            halt();
        }
        else if (turn_cnt >= count_down_CNT) {
            observe();
            for (int i = 0; i < len(mtr_input); i++) {
                const MotorInputData datum = sitDown(-turn_cnt, mtr_input[i].get());
                mtr_input[i].pull(datum);
                mtr_input[i].pack(&tx_msg[i]);
            }
            turn_cnt++;
        }
        break;
    default:
        printf("%% Undefined mode %%\n");
        mode = SetzeroMode;
        turn_cnt = -2;
        halt();
        break;
    }

#if 0
    overwatch();
#endif
    for (int i = 0; i < 3; i++) {
        can1.write(tx_msg[i]);
    }
    for (int i = 3; i < 6; i++) {
        can2.write(tx_msg[i]);
    }
}

void interact()
{
    int ch = '\0';
#if USE_PID
    bool pid_okay = true;
#endif
    while (pc.readable()) {
        ch = pc.getc();
        if (mode == ReadcmdMode) {
            const bool entered = putChar(ch);
            if (entered) {
                turn_cnt = -2;
                console();
                io_input = NULL;
                clearBuffer();
            }
            else {
                turn_cnt = -2;
                mode = ReadcmdMode;
            }
        }
        else {
            switch (ch) {
            case ESC:
                printf("\n%% Exiting motor mode %%\n");
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
                printf("\n%% Entering motor mode %%\n");
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
                printf("\n%% Set zero %%\n");
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
                            printf("\n%% Motor%c rest position %%\n", ch);
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
                printf("\n%% Run %%\n");
                mode = RuntimeMode;
                turn_cnt = 0;
#if USE_PID
                pid_okay &= pidInit();
                if (!pid_okay) {
                    printf("%% Initializing PID failed %%\n");
                    mode = SetzeroMode;
                    turn_cnt = -2;
                }
#endif
                for (int i = 0; i < len(mtr_input); i++) {
                    mtr_input[i].init();
                }
                return;
            case 'o':
                printf("\n%% Observe %%\n");
                mode = ObserveMode;
                turn_cnt = -2;
                return;
            case 'b':
                printf("\n%% Break %%\n");
                mode = SetzeroMode;
                turn_cnt = -2;
                halt();
                return;
            case '.':
                printf("\n%% Sit down %%\n");
                mode = GotobedMode;
                turn_cnt = count_down_CNT;
                for (int i = 0; i < len(mtr_input); i++) {
                    mtr_input[i].init();
                }
                return;
            case 'l':
                if (mode == SetzeroMode && turn_cnt < 0) {
                    printf("\n%% Listen %%\n");
                    mode = ReadcmdMode;
                    turn_cnt = -2;
                }
                return;
            }
            for (int i = 0; i < 3; i++) {
                can1.write(tx_msg[i]);
            }
            for (int i = 3; i < 6; i++) {
                can2.write(tx_msg[i]);
            }
        }
    }
}

int console()
{
    int result   = 0;
    char lhs[16] = {};
    int  no      = 0;
    float rhs    = 0.0;
    char op[16]  = {};
    if (io_input == NULL) {
        printf("\n%% Leaving listening mode %%\n");
        mode = SetzeroMode;
        result = 0;
    }
    else {
        int sscanf_res = 0;
        for (int i = 0; i < len(lhs); i++) {
            lhs[i] = '\0';
        }
        if (sscanf_res == 3) {
            if (no > len(pids) || no <= 0) {
                result = -1;
            }
            else {
                if (areSameStr("Kp", lhs)) {
                    pids[no - 1].Kp = rhs;
                    result = 1;
                }
                else if (areSameStr("Ki", lhs)) {
                    pids[no - 1].Ki = rhs;
                    result = 1;
                }
                else if (areSameStr("Kd", lhs)) {
                    pids[no - 1].Kd = rhs;
                    result = 1;
                }
                else {
                    result = -1;
                }
            }
            for (int i = 0; i < len(op); i++) {
                op[i] = '\0';
            }
            sscanf_res = sscanf(io_input, "%s", op);
            if (sscanf_res == 1) {
                if (areSameStr(op, "jump")) {
                    operation = jump;
                    result = 1;
                }
                else if (areSameStr(op, "jump1")) {
                    operation = jump1;
                    result = 1;
                }
                else if (areSameStr(op, "standup")) {
                    operation = standUp;
                    result = 1;
                }
                else {
                    result = -1;
                }
            }
            else {
                result = -1;
            }
        }
        else {
            result = -1;
        }
    }
    if (result > 0) {
        printf("Command well recieved\n");
    }
    else if (result < 0) {
        printf("Unknown command or wrong command\n");
    }
    return result;
}
