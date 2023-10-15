#include "capstone.h"

static char my_buffer[64];
static int my_cursor = 0;

char *callIO(char ch)
{
    switch (ch) {
    default:
        my_buffer[my_cursor++] = ch;
        pc.putc(ch);
        return NULL;
    case '\n':
        my_buffer[my_cursor] = '\0';
        my_cursor = 0;
        printf("%s\n", my_buffer);
        io_input = my_buffer;
        return my_buffer;
    case '\r':
        my_cursor = 0;
        my_buffer[0] = '\0';
        printf("\r");
        return NULL;
    case 8:
        my_buffer[my_cursor--] = '\0';
        printf("\r%s", my_buffer);
        return NULL;
    case 27:
        my_buffer[my_cursor] = '\0';
        my_cursor = 0;
        return NULL;
    }
}
