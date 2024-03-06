#include "capstone.hpp"

#define mkCANManager(rd,td,transceiver) (CANManager(rd, td, transceiver, len(transceiver)))

static void             onMsgReceived1(void);
static void             onMsgReceived2(void);
static void             transmitMsg(void);

static void             start(void);
static void             halt(void);
static void             observe(void);

static bool             loadRefTbl1(bool until);

static void             jump(void);
static void             standUp(void);
#if USE_PID
static void             jump1(void);
static void             standUp1(void);
#endif
static void             serial_isr(void);

static void             interact(void);
static void             prompt(const char *msg);

static int              id_to_index(int id);

#if USE_PID
static void             pidInit(void);
static void             pidCompute(void);
#endif
static Motor::PutData   sitDown_calc(int count_down, const Motor::PutData &datum);

IO          terminal;
Timer       timer;
Ticker      send_can;
Serial      pc(PA_2, PA_3);

Mode        mode                = SetzeroMode;
long int    turn_cnt            = -2;
void        (*operation)(void)  = standUp;
const int   count_down_MAX_CNT  = -100;
#if USE_PID
long int    PID_START_TICK      = 390;
#endif

MotorHandler motor_handlers[] = {
#if USE_PID
    //           #  Kp    Ki    Kd
    MotorHandler(1, 1.30, 0.10, 0.00), // SET ME !!!
    MotorHandler(2, 1.25, 0.30, 0.00), // SET ME !!!
    MotorHandler(3, 2.00, 1.00, 0.00), // SET ME !!!
    MotorHandler(4, 1.30, 0.10, 0.00), // SET ME !!!
    MotorHandler(5, 1.25, 0.30, 0.00), // SET ME !!!
    MotorHandler(6, 2.00, 1.00, 0.00), // SET ME !!!
#else
    //           #
    MotorHandler(2), // SET ME !!!
    MotorHandler(3), // SET ME !!!
    MotorHandler(5), // SET ME !!!
    MotorHandler(6), // SET ME !!!
#endif
};

#if USE_PID
MotorHandler    *transceiver1[] = { &motor_handlers[0], &motor_handlers[1], &motor_handlers[2], }; // SET ME !!!
MotorHandler    *transceiver2[] = { &motor_handlers[3], &motor_handlers[4], &motor_handlers[5], }; // SET ME !!!
#else
MotorHandler    *transceiver1[] = { &motor_handlers[0], &motor_handlers[1], }; // SET ME !!!
MotorHandler    *transceiver2[] = { &motor_handlers[2], &motor_handlers[3], }; // SET ME !!!
#endif

CANManager      cans[] = { mkCANManager(PB_8, PB_9, transceiver1), mkCANManager(PB_5, PB_6, transceiver2), }; // SET ME !!!
void            (*const onMsgReceived[])(void) = { onMsgReceived1, onMsgReceived2, }; // SET ME !!!

int main(void)
{
    pc.baud(921600);
    pc.attach(interact);

    for (int i = 0; i < len(cans); i++) {
        cans[i].init(0x01 << 21, 0xFFE00004, onMsgReceived[i]);
    }

    for (int i = 0; i < len(motor_handlers); i++) {
        const Motor::PutData init_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };
        motor_handlers[i].data_into_motor = init_data;
    }
    transmitMsg();

    printf("\n");
    printf("\r<< %s >>\n", CAPSTONE);
    printf("\rVERSION = %s\n", VERSION);
#if USE_PID
    printf("\rUSE_PID = true\n");
#else
    printf("\rUSE_PID = false\n");
#endif
    printf("\rRUNTIME_TICK_MAX = %d\n", RUNTIME_TICK_MAX);
    printf("\rTick_dt = %lf[s]\n", Tick_dt);
    printf("\n");

    turn_cnt = -2;
    terminal.setPrompt(prompt);
    timer.start();
    send_can.attach(serial_isr, Tick_dt);
}

void onMsgReceived1()
{
    cans[0].onMsgReceived();
}

void onMsgReceived2()
{
    cans[1].onMsgReceived();
}

void transmitMsg()
{
    for (int i = 0; i < len(cans); i++) {
        cans[i].sendMsg();
    }
}

void start()
{
    return;
}

void halt()
{
    const Motor::PutData zero_data = { .p = 0.0, .v = 0.0, .kp = 0.0, .kd = 0.0, .t_ff = 0.0 };

    for (int i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].data_into_motor = zero_data;
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
            const Motor::GetData data = motor_handlers[i].data_from_motor; // SENSITIVE POINT
            const int id = motor_handlers[i].motor_id;
            printf("\rtheta%d(%ld) = %f; omega%d(%ld) = %f;\n", id, row, data.p, id, row, data.v);
        }
        printf("\n");
    }
}

bool loadRefTbl1(const bool until)
{
    static Motor::PutData last_data[len(motor_handlers)];

    if ((turn_cnt < len(reftbl1)) && until) {
        for (int i = 0; i < len(motor_handlers); i++) {
            motor_handlers[i].data_into_motor = reftbl1[turn_cnt][(motor_handlers[i].id() - 1) % 3]; // SENSITIVE POINT
            last_data[i] = motor_handlers[i].data_into_motor;
        }
        return true;
    }
    else {
        for (int i = 0; i < len(motor_handlers); i++) {
            motor_handlers[i].data_into_motor = last_data[i];
        }
        return false;
    }
}

void jump()
{
    loadRefTbl1(turn_cnt < len(reftbl1));
}

void standUp()
{
    const unsigned char lines[3][8] = {
        { 0x7C, 0xA5, 0x96, 0xB0, 0x7A, 0x99, 0x97, 0xFF, },
        { 0x7C, 0xED, 0x96, 0xB0, 0x7A, 0x99, 0x95, 0xC6, },
        { 0x7F, 0xBA, 0x7F, 0xF0, 0x39, 0x00, 0x07, 0x8D, },
    };

    for (int i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].data_into_motor = decode16(&lines[(motor_handlers[i].id() - 1) % 3]); // SENSITIVE POINT
    }
}

#if USE_PID
void jump1()
{
    loadRefTbl1(turn_cnt <= PID_START_TICK);
    if (turn_cnt == PID_START_TICK) {
        pidInit();
    }
    else if (turn_cnt > PID_START_TICK) {
        pidCompute();
    }
}

void standUp1()
{
    if (turn_cnt < PID_START_TICK) {
        standUp();
        return;
    }
    if (turn_cnt == PID_START_TICK) {
        standUp();
        pidInit();
        return;
    }
    if (turn_cnt > PID_START_TICK) {
        pidCompute();
        return;
    }
}
#endif

void serial_isr()
{
    switch (mode) {
    case RuntimeMode:
        if (turn_cnt > RUNTIME_TICK_MAX) {
            turn_cnt = -2;
            halt();
        }
        else if (turn_cnt >= 0) {
            operation();
            observe();
            turn_cnt++;
        }
        for (int i = 0; i < len(motor_handlers); i++) {
            motor_handlers[i].send_msg();
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
                const Motor::PutData datum = sitDown_calc(-turn_cnt, motor_handlers[i].data_into_motor);
                motor_handlers[i].send_msg();
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

    transmitMsg();
}

void interact()
{
    int ch = 0;

    if (mode == ReadcmdMode) {
        const bool prompt_routine_breaked = terminal.runPrompt();
        if (prompt_routine_breaked) {
            mode = SetzeroMode;
        }
        turn_cnt = -2;
        return;
    }

    ch = IO::getc();

    if (special_key_flag == NOT_A_SPECIAL_KEY) {
        switch (ch) {
        case ESC:
            printf("\n\r%% Exiting motor mode %%\n");
            for (int i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, } };
                motor_handlers[i].put_txmsg(msg);
            }
            turn_cnt = -2;
            return;
        case 'm':
            printf("\n\r%% Entering motor mode %%\n");
            for (int i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, } };
                motor_handlers[i].put_txmsg(msg);
            }
            turn_cnt = -2;
            return;
        case 'z':
            printf("\n\r%% Set zero %%\n");
            for (int i = 0; i < len(motor_handlers); i++) {
                const UCh8 msg = { .data = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, } };
                motor_handlers[i].put_txmsg(msg);
            }
            turn_cnt = -2;
            return;
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
            for (int i = 0; i < len(motor_handlers); i++) {
                if (motor_handlers[i].id() + '0' == ch) { // SENSITIVE POINT
                    const UCh8 msg = { .data = { 0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF, } };
                    printf("\n\r%% Motor #%c rest position %%\n", ch);
                    motor_handlers[i].put_txmsg(msg);
                    break;
                }
            }
            return;
        case 'r':
            printf("\n\r%% Run %%\n");
            mode = RuntimeMode;
            turn_cnt = 0;
            start();
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
        case '.':
            printf("\n\r%% Sit down %%\n");
            mode = SitdownMode;
            turn_cnt = count_down_MAX_CNT;
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
    else {
        return;
    }
}

void prompt(const char *const msg)
{
    char var_name[16];
    char op_name[16];
    int sscanf_res = 0;
    int pid_start_tick = 0;
    int motor_id = 0;
    float value = 0.0;
    bool res = false;

    if (msg == NULL) {
        printf("\n\r%% Leaving listening mode %%\n");
        mode = SetzeroMode;
        return;
    }

#if USE_PID
    sscanf_res = sscanf(msg, "%s %d = %f", var_name, &motor_id, &value);
    if (sscanf_res == 3) {
        const int idx = id_to_index(motor_id);
        if (idx < 0) {
            res = false;
            goto RET;
        }
        else if (areSameStr("Kp", var_name)) {
            motor_handlers[idx].set_Kp(value); // SENSITIVE POINT
            res = true;
            goto RET;
        }
        else if (areSameStr("Ki", var_name)) {
            motor_handlers[idx].set_Ki(value); // SENSITIVE POINT
            res = true;
            goto RET;
        }
        else if (areSameStr("Kd", var_name)) {
            motor_handlers[idx].set_Kd(value); // SENSITIVE POINT
            res = true;
            goto RET;
        }
        else {
            res = false;
            goto RET;
        }
    }

    sscanf_res = sscanf(msg, "pid start tick = %d", &pid_start_tick);
    if (sscanf_res == 1) {
        if (PID_START_TICK >= 0) {
            PID_START_TICK = pid_start_tick;
            res = true;
            goto RET;
        }
        else {
            res = false;
            goto RET;
        }
    }
#endif

    sscanf_res = sscanf(msg, "%s", op_name);
    if (sscanf_res == 1) {
        if (areSameStr(op_name, "skip")) {
            res = true;
            goto RET;
        }
        else if (areSameStr(op_name, "jump")) {
            operation = jump;
            res = true;
            goto RET;
        }
        else if (areSameStr(op_name, "standUp")) {
            operation = standUp;
            res = true;
            goto RET;
        }
#if USE_PID
        else if (areSameStr(op_name, "jump1")) {
            operation = jump1;
            res = true;
            goto RET;
        }
        else if (areSameStr(op_name, "standUp1")) {
            operation = standUp1;
            res = true;
            goto RET;
        }
#endif
        else {
            res = false;
            goto RET;
        }
    }

RET:
    if (res) {
        printf("\n\rCommand well recieved\n");
    }
    else {
        printf("\n\rUnknown command or wrong command\n");
    }
}

int id_to_index(const int id)
    // finds the index of the motor #`id` in `motor_handlers`
{
    for (int i = 0; i < len(motor_handlers); i++) {
        if (motor_handlers[i].id() == id) {
            return i;
        }
    }
    return -1;
}

#if USE_PID
void pidInit()
{
    for (int i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].pidInit();
    }
}

void pidCompute()
{
    for (int i = 0; i < len(motor_handlers); i++) {
        motor_handlers[i].pidCompute();
    }
}
#endif

Motor::PutData sitDown_calc(const int count_down, const Motor::PutData &datum)
{
    const Motor::PutData res = {
        .p    = (datum.p    * abs(count_down)) / abs(count_down_MAX_CNT),
        .v    = (datum.v    * abs(count_down)) / abs(count_down_MAX_CNT),
        .kp   = (datum.kp   * abs(count_down)) / abs(count_down_MAX_CNT),
        .kd   = (datum.kd   * abs(count_down)) / abs(count_down_MAX_CNT),
        .t_ff = (datum.t_ff * abs(count_down)) / abs(count_down_MAX_CNT),
    };

    return res;
}
