#ifndef can_h
#define can_h

#include <Arduino.h>

enum frame_type {
    FRAME_DATA,   // Data frame
    FRAME_RTR     // Remote Transmission Request frame
};

struct message {
    uint32_t id;          // CAN identifier
    bool extended;  // Extended frame flag (true for extended, false for standard)
    uint8_t data[8];     // Data bytes (up to 8 bytes)
    enum frame_type type; // Type of frame (data or RTR)
    bool single_shot;    // Single shot transmission flag
};

//function definitions
void can_init();
void can_send_frame(uint32_t id, const uint8_t data[8],enum frame_type type, bool single_shot);

#endif // can_h
