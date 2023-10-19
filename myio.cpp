#include "capstone.h"

static char my_buffer[64];
static int my_cursor = 0;

void clear_my_buffer(void)
{
    int i = 0;
    for (i = 0; i < sizeof(my_buffer)/sizeof(my_buffer[0]); i++) {
        my_buffer[i] = '\0';
    }
    my_cursor = 0;
}

bool receivech(char ch)
{
    switch (ch) {
    default:
        if (my_cursor < 0) {
            io_input = NULL;
            return true;
        }
        else if (my_cursor < sizeof(my_buffer) / sizeof(my_buffer[0])) {
            my_buffer[my_cursor++] = ch;
            printf("\r%s", my_buffer);
            io_input = NULL;
            return false;
        }
        else {
            io_input = my_buffer;
            my_cursor = 0;
            return true;
        }       
    case '\n':
    case '\r':
        printf("\n[ECHO] %s\n\r", my_buffer);
        io_input = my_buffer;
        return true;
    case '\0':
        return false;
    case '\b':
        if (my_cursor > 0) {
            my_buffer[--my_cursor] = '\0';
            printf("\r%s", my_buffer);
        }
        else {
            my_buffer[0] = '\0';
            printf("\r");
        }
        io_input = NULL;
        return false;
    case ESC:
        clear_my_buffer();
        printf("\n");
        io_input = NULL;
        return true;
    }
}
