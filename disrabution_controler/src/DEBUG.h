#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>

//var defines
enum DEBUG_LEVELS {
    DEBUG_FULL,
    DEBUG_PARTIAL_IO,
    DEBUG_NO_DEBUG,
    DEBUG_PARTIAL_USB
};


//function define
void debug_msg(enum DEBUG_LEVELS debug_level, const char* debug_message, bool include_num, int num_include);
void debug_level_set(enum DEBUG_LEVELS new_debug_level);


#endif // DEBUG_H