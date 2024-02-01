#include "capstone.h"

void IO::set_prompt(void (*const prompt)(const char *msg))
{
    this->prompt = prompt;
    this->clear();
}

bool IO::run_prompt()
{
    char *msg    = NULL;
    int  ch      = '\0';
    bool entered = false;

    while (1) {
        ch = getc();
        if (ch == 0) {
            return false;
        }
        if (ch == ESC) {
            return true;
        }
        entered = takech(ch);
        if (entered) {
            sync(msg);
            prompt(msg);
            clear();
            return true;
        }
    }
}

int IO::getc()
{
    if (pc.readable()) {
        return pc.getc();
    }
    else {
        return 0;
    }
}

void IO::clear()
{
    for (int i = 0; i < len(buffer); i++) {
        buffer[i] = '\0';
    }
    cursor = 0;
    theend = 0;
}

bool IO::takech(const int ch)
{
    switch (ch) {
    default:
        if (cursor < 0) {
            result = NULL;
            return false;
        }
        if (cursor > theend) {
            result = NULL;
            return false;
        }
        if (theend + 1 >= len(buffer)) {
            result = NULL;
            return false;
        }
        for (int i = theend; i >= cursor; i--) {
            buffer[i + 1] = buffer[i];
        }
        buffer[cursor++] = ch;
        buffer[++theend] = '\0';
        print();
        result = NULL;
        return false;
    case '\b':
        if (cursor > theend) {
            result = NULL;
            return false;
        }
        if (theend + 1 >= len(buffer)) {
            result = NULL;
            return false;
        }
        if (cursor <= 0) {
            result = NULL;
            return false;
        }
        for (int i = --cursor; i < theend; i++) {
            buffer[i] = buffer[i + 1];
        }
        if (theend > 0) {
            buffer[theend--] = '\0';
        }
        print();
        result = NULL;
        return false;
    case '\n':
    case '\r':
        result = buffer;
        return true;
    case '\0':
        return false;
    case ESC:
        clear();
        printf("\n");
        result = NULL;
        return true;
    case 224:
        if (theend + 1 >= len(buffer)) {
            result = NULL;
            return false;
        }
        switch (getc()) {
        case LEFT_DIRECTION:
            if (cursor > 0) {
                cursor--;
            }
            print();
            result = NULL;
            return false;
        case RIGHT_DIRECTION:
            if (cursor < theend) {
                cursor++;
            }
            print();
            result = NULL;
            return false;
        }
    }
    return false;
}

void IO::sync(char *&msg)
{
    msg = result;
}

void IO::print()
{
    int i = 0;

    printf("\r");
    for (i = 0; i < len(buffer); i++) {
        printf(" ");    
    }
    printf("\r");
    for (i = 0; i < cursor; i++) {
        printf(" ");
    }
    for (i = cursor; i < theend; i++) {
        printf("%c", buffer[i]);
    }
    buffer[i] = '\0';
    printf("\r");
    for (int i = 0; i < cursor; i++) {
        printf("%c", buffer[i]);
    }
    std::cout.flush();
}

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

bool areSameStr(const char *const lhs, const char *const rhs)
    // returns whether the two astrings are the same
{
    if (lhs == NULL || rhs == NULL) {
        return false;
    }
    return strcmp(lhs, rhs) == 0;
}

bool inRange(const float left, const float x, const float right)
    // returns wheter x is in the interval [left, right]
{
    return (left <= x) && (x <= right);
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
