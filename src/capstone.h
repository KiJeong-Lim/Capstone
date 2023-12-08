#ifndef CAPSTON_H
#define CAPSTON_H

/***** INCLUDES *****/
/* standard C++ library */
#include <cstdio>
#include <cstdlib>
#include <cstring>
/* NUCLEO-F446RE */
#include "mbed.h"
/* our headers */
#include "version.h"
#include "constants.h"
#include "klasses.hpp"

/***** MACROS *****/
#define NumberOfMotors  3
#define len(xs)         (sizeof(xs) / sizeof(*(xs)))
#define max(x, y)       ((x) >= (y)? (x): (y))
#define min(x, y)       ((y) >= (x)? (x): (y))

/***** VARIABLE DECLARTIONS *****/
/* reftbl0.cpp */
extern const MotorInputData refs_tbl[1000][3];
/* setup.cpp */
extern int              turn_cnt;
extern char const       *io_input;
extern Timer            timer;
extern CANMessage       rx_msg, tx_msg[NumberOfMotors];
extern Serial           pc;
extern CAN              can;
extern Ticker           send_can;
extern MotorInput       mtr_input[NumberOfMotors];
extern MotorOutput      mtr_output[NumberOfMotors];

/***** FUNCTION PROTOTYPES *****/
/* cancomm.cpp */
MotorInputData  decode16(const unsigned char (*input_data)[8]);
void            onMsgReceived(void);
/* loop.cpp */
void            serialIsr(void);
void            interact(void);
/* setup.cpp */
int             main(void);
/* teratermcomm.cpp */
bool            putChar(char ch);
void            clearBuffer(void);
/* utilities.cpp */
void            limitNorm(float &x, float &y, float limit);
unsigned int    floatToUint(float x, float x_min, float x_max, int bits);
float           uintToFloat(int x_int, float x_min, float x_max, int bits);
float           middle(float x, float y, float z);
float           getTime(void);
bool            areSameStr(const char *str1, const char *str2);
bool            inRange(float left, float x, float rhs);

#endif
