#ifndef VERSION
#define VERSION "2.8.4 (2024-03-16 00:00)"

/* VERSION=2.6.0
Modify: roll back functions `float2uint` and `uint2float`
    -- Details:
        Before:
            unsigned int    float2uint(float x, float x_min, float x_max, int bits);
            float           uint2float(unsigned int x_int, float x_min, float x_max, int bits);
        After:
            int             float2uint(float x, float x_min, float x_max, int bits);
            float           uint2float(int x_int, float x_min, float x_max, int bits);
*/

/* VERSION=2.7.0
Change: Rename `float2uint` -> `float2int`, `uint2float` -> `int2float`
Add: function `standUp2`
*/

/* VERSION=2.7.1
Fix: Remove warning "single-precision operand implicitly converted to double-precision" by changing 0.25 -> 0.25f
    -- Details:
        void standUp2()
        {
            standUp();

            for (int i = 0; i < len(motor_handlers); i++) {
                motor_handlers[i].data_into_motor.p *= 0.25f;
                motor_handlers[i].data_into_motor.v *= 0.25f;
                motor_handlers[i].data_into_motor.kp *= 0.25f;
                motor_handlers[i].data_into_motor.kd *= 0.25f;
                motor_handlers[i].data_into_motor.t_ff *= 0.25f;
            }
        }
Modify: Change the default operation
    -- Details:
        Before:
            void (*operation)(void) = standUp;
        After:
            void (*operation)(void) = standUp2;
Modify: add command `standUp2` in function `prompt` in `main.cpp`
*/

/* VERSION=2.7.2
Add: variable `debug` in `main.cpp`
    -- Details: static bool debug = false;
Remove: macro constant `DEBUG_TXMSG` in `capstone.hpp`
Modify: add command `debug` in function `prompt` in `main.cpp`
*/

/* VERSION=2.7.3
Modify: Rename: `int2float` -> `uint2float`, `float2int` -> `float2uint`
Remove: functions `encode16`, `decode16`
Add: functions `encodetx`, `decodetx`, `encoderx`
Add: struct `GetDataWithId`
    -- Details: struct GetDataWithId { int motor_id; float p; float v; float i; };
Add: Global Declartion of variables: `terminal`, `send_can`
*/

/* VERSION=2.7.4
Modify: Add the `volatile` type qualifier in `PIDController`
*/

#endif
