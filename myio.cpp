#include "capstone.h"

static char my_buffer[64];
static int my_cursor = 0;

bool callIO(char ch)
{
    switch (ch) {
    default:
        if (my_cursor < sizeof(my_buffer) / sizeof(my_buffer[0])) {
            my_buffer[my_cursor++] = ch;
            pc.putc(ch);
        }
        return false;
    case '\n':
        my_buffer[my_cursor] = '\0';
        my_cursor = 0;
        printf("\n[echo] %s\n\r", my_buffer);
        io_input = my_buffer;
        return true;
    case '\r':
        my_cursor = 0;
        my_buffer[0] = '\0';
        printf("\r");
        return false;
    case 8:
        if (my_cursor > 0) {
            my_buffer[--my_cursor] = '\0';
            printf("\r%s", my_buffer);
        }
        else {
            my_buffer[0] = '\0';
            printf("\r");
        }
        return false;
    case 27:
        my_cursor = 0;
        my_buffer[0] = '\0';
        printf("\n");
        return false;
    }
}
