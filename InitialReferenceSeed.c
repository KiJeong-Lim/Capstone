#include <stdio.h>
#include <math.h>

#define txMsg1 (txMsg[0])
#define txMsg2 (txMsg[1])
#define txMsg3 (txMsg[2])

typedef struct TxMsg { int txMsg; double p; double v; double kp; double kd; double t_ff; } txMsg_t;

void loop(int cnt);

FILE *file = NULL;

txMsg_t txMsg[3] =
    { { .txMsg = 1, }
    , { .txMsg = 2, }
    , { .txMsg = 3, }
    };

void pack_cmd(txMsg_t *msg, double p, double v, double kp, double kd, double t_ff)
{
    msg->p = p;
    msg->v = v;
    msg->kp = kp;
    msg->kd = kd;
    msg->t_ff = t_ff;
}

void loop(int x)
{
#if 1
    if (0<x && x<99) {
        pack_cmd(&txMsg1, 0, 0, 0, 0, 0);
        pack_cmd(&txMsg2, 0, 0, 0, 0, 0);
        pack_cmd(&txMsg3, -0.20, 0, 4, 3, 0);
    } else if(99<x && x<199) {
        pack_cmd(&txMsg1, -0.1, 0, 18, 3.5, 0);
        pack_cmd(&txMsg2, -0.115, 0, 18, 3.5, 0);
        pack_cmd(&txMsg3, 0, 0, 15, 3, 0);
    } else if(199<x && x<390) {
        pack_cmd(&txMsg1, -0.11, 0, 10, 0.9, 0);
        pack_cmd(&txMsg2, -0.1125, 0, 10, 1.5, -2);
        pack_cmd(&txMsg3, -0.1, 0, 10, 1, 0);
    } else if(389<x && x<400) {
        pack_cmd(&txMsg2, -0.2925, 0, 20, 0.1, -2);
    } else if(399<x && x<405) {
        pack_cmd(&txMsg1, -0.1, 0, 10, 0.5, 0);
        pack_cmd(&txMsg2, -0.18, 0, 16, 0, 2);
        pack_cmd(&txMsg3, -0.175, 0, 15, 0.5, 0);
    } else if(404<x && x<414){
        pack_cmd(&txMsg1, -0.175, 0, 15, 0.5, 0);
    } else if(413<x && x<415){
        pack_cmd(&txMsg2, 0, 0.1, 15, 0, 2);
    } else if(414<x && x<420){
        pack_cmd(&txMsg1, -0.175, 0, 10, 1, 0);
        pack_cmd(&txMsg3, 0, 0, 12, 0.3, 0);
    } else if(419<x && x<424){
        pack_cmd(&txMsg1, 0, 0, 10, 0.1, 0);
    } else if(423<x && x<425){
        pack_cmd(&txMsg2, -0.3, 0, 15, 0, -2);
    } else if(424<x && x<430){
        pack_cmd(&txMsg3, -0.225, 0, 10, 1, 0);
    } else if(429<x && x<435){
        pack_cmd(&txMsg1, -0.1125, 0, 10, 1, 0);
        pack_cmd(&txMsg2, -0.15, 0, 10, 0, -2);
    } else if(434<x && x<440){
        pack_cmd(&txMsg2, -0.11, 0, 10, 2.7, -2);
    } else if(439<x && x<450){
        pack_cmd(&txMsg3, -0.0375, 0, 10, 1, 0);
    } else if(600<x && x<700){
        pack_cmd(&txMsg1, 0, 0, 4, 3.5, 0);
        pack_cmd(&txMsg2, 0, 0, 2.5, 4.5, -4);
        pack_cmd(&txMsg3, 0, 0, 10, 1.5, 0);
    } else if(700<x && x<750){
        pack_cmd(&txMsg1, 0, 0, 0, 0.1, 0);
        pack_cmd(&txMsg2, 0, 0, 0, 0.1, 0);
        pack_cmd(&txMsg3, 0, 0, 0, 0.1, 0);
    }
#elif 0
    float const omega = 0.1;
    float const dt = 0.01;
    pack_cmd(&txMsg1, sin(omega * x * dt), 0, 0, 0.1, 0);
#endif
}

void serial_isr()
{
    int x = 0, i = 0;

    for (x = 0; x < 1000; x++) {
        loop(x);
        for (i = 0; i < sizeof(txMsg) / sizeof(txMsg[0]); i++) {
            fprintf(file, "%lf\t", txMsg[i].p);
            fprintf(file, "%lf\t", txMsg[i].v);
            fprintf(file, "%lf\t", txMsg[i].kp);
            fprintf(file, "%lf\t", txMsg[i].kd);
            fprintf(file, "%lf\t", txMsg[i].t_ff);
        }
        fprintf(file, " where x = %d\n", x);
    }
}

int main(void)
{
    char file_name[32];

    printf("I'm a program to generate a copy of the initial reference table.\n");
    printf("Enter the name of the copy file: ");
    fflush(stdout);
    scanf("%s", file_name);
    printf("The given file name was: %s\n", file_name);

    file = fopen(file_name, "w+");

    if(file != NULL)
    {
        serial_isr();
        fclose(file);
        printf("Generated.\n");
    }

    file = NULL;
    return 0;
}
