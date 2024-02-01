#include "capstone.h"

void limitNorm(float &x, float &y, const float limit)
    // scales the lenght of vector (x, y) to be <= limit
{
    const float norm = sqrt(x * x + y * y);
    if (norm > limit) {
        x *= limit / norm;
        y *= limit / norm;
    }
}

unsigned int floatToUint(const float x, const float x_min, const float x_max, const int bits)
    // converts a float to an unsigned int, given range and number of bits
{
    const float span = x_max - x_min;
    const float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uintToFloat(const int x_int, const float x_min, const float x_max, const int bits)
    // converts unsigned int to float, given range and number of bits
{
    const float span = x_max - x_min;
    const float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

float middle(const float x, const float y, const float z)
    // returns the 2nd largest value among x, y and z
{
	if (y <= x && x <= z || z <= x && x <= y) {
        return x;
    }
    if (x <= y && y <= z || z <= y && y <= x) {
        return y;
	}
    return z;
}

float getTime()
    // returns the current time
{
    return (float)(timer.read());
}

bool areSameStr(const char (*const lhs), const char (*const rhs))
    // returns whether the two astrings are the same
{
    if (lhs == NULL || rhs == NULL) {
        return false;
    }
    return strcmp(lhs, rhs) == 0;
}

bool inRange(const float left, const float x, const float right)
{
    return left <= x && x <= right;
}

Gear::Gear(const int gear)
    : gear(gear)
    , gear_cnt(0)
{
}

Gear::Gear(const Gear &other)
    : gear(other.gear)
    , gear_cnt(other.gear_cnt)
{
}

Gear::~Gear()
{
}

bool Gear::go()
{
    if ((gear_cnt + 1) % gear == 0) {
        gear_cnt = 0;
        return true;
    }
    if (gear_cnt >= 0) {
        gear_cnt++;
    }
    else {
        gear_cnt = 0;
    }
    return false;
}

void Gear::reset()
{
    gear_cnt = 0;
}
