

#ifndef DEBUG_cpp
#define DEBUG_cpp

#include <Arduino.h>
#include "DEBUG.h"

//var defines
enum DEBUG_LEVELS debug_level = DEBUG_NO_DEBUG;

//functions

/**
 * @brief Prints a debug message with optional attached value.
 * 
 * This function prints a debug message along with the current millis() value. 
 * The debug message can be customized based on the debug level provided.
 * 
 * @param message_debug_level The debug level for the message.
 * @param debug_message The debug message to be printed.
 * @param include_num Flag indicating whether to include an attached value.
 * @param num_include The attached value to be printed (if include_num is true).
 */
void IRAM_ATTR debug_msg(enum DEBUG_LEVELS message_debug_level, const char* debug_message, bool include_num, int num_include) {
    if (debug_level != DEBUG_NO_DEBUG) {
        switch (message_debug_level) {
            case DEBUG_FULL:
                Serial.print("debug msg at millis: ");
                Serial.print(millis());
                Serial.print("| ");
                if (include_num) {
                    Serial.print(debug_message);
                    Serial.print("  | attached value: ");
                    Serial.println(num_include);
                } else {
                    Serial.println(debug_message);
                }
                break;
            case DEBUG_PARTIAL_IO:
                Serial.print("debug msg at millis: ");
                Serial.print(millis());
                Serial.print("| ");
                if (include_num) {
                    Serial.print(debug_message);
                    Serial.print("  | attached value: ");
                    Serial.println(num_include);
                } else {
                    Serial.println(debug_message);
                }
                break;
            case DEBUG_PARTIAL_USB:
                Serial.print("debug msg at millis: ");
                Serial.print(millis());
                Serial.print("| ");
                if (include_num) {
                    Serial.print(debug_message);
                    Serial.print("  | attached value: ");
                    Serial.println(num_include);
                } else {
                    Serial.println(debug_message);
                }
                break;
            case DEBUG_NO_DEBUG:
                break;
        }
    }
}

void debug_level_set(enum DEBUG_LEVELS new_debug_level) {
    debug_level = new_debug_level;
}



#endif // DEBUG_cpp