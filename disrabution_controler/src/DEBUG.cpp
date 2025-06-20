

#ifndef DEBUG_cpp
#define DEBUG_cpp

#include <Arduino.h>
#include "DEBUG.h"

//var defines
enum DEBUG_LEVELS debug_level = DEBUG_NO_DEBUG;

//functions


/**
 * @brief Prints a debug message to the Serial output if the specified debug level is enabled.
 * 
 * This function is intended to be called from an interrupt context (IRAM_ATTR).
 * It prints a timestamp, the provided debug message, and optionally an attached value.
 * 
 * @tparam T The type of the value to be included with the debug message.
 * @param message_debug_level The debug level of this message (of type DEBUG_LEVELS).
 * @param debug_message The message string to print.
 * @param include_num If true, the value of num_include will be printed after the message.
 * @param num_include The value to print if include_num is true.
 * 
 * @note The function will not print anything if either the global debug_level or the message_debug_level is DEBUG_NO_DEBUG.
 * @note Only messages with a level equal to or lower than the current debug_level will be printed.
 */
template<typename T>
void IRAM_ATTR debug_msg(enum DEBUG_LEVELS message_debug_level, const char* debug_message, bool include_num, T num_include) {
    if (debug_level == DEBUG_NO_DEBUG || message_debug_level == DEBUG_NO_DEBUG) {
        return;
    }

    // Only print if the message level is enabled by the current debug level
    if (debug_level == message_debug_level || debug_level == DEBUG_FULL) {
        Serial.print("debug msg at millis: ");
        Serial.print(millis());
        Serial.print("| ");
        Serial.print(debug_message);
        if (include_num) {
            Serial.print("  | attached value: ");
            Serial.println(num_include);
        } else {
            Serial.println();
        }
    }
}

void debug_level_set(enum DEBUG_LEVELS new_debug_level) {
    debug_level = new_debug_level;
}



#endif // DEBUG_cpp