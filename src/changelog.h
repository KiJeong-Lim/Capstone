#ifndef VERSION
#define VERSION "2.7.0 (2024-03-08 18:00)"

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
Rename: `float2uint` -> `float2int`, `uint2float` -> `int2float`
Add: function `standUp2`
*/

#endif
