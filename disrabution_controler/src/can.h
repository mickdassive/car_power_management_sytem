#ifndef can_h
#define can_h

#include <Arduino.h>

struct can_module_data {
    uint32_t bus_id;
    unsigned long last_heartbeat;
    bool is_alive;
    bool if_faulted;
};

//addresses and commands for CAN controller
static can_module_data can_module_brodcast = {0x00000000, 0, false, false};
static can_module_data can_module_disrabution_controler = {0x00000001, 0, true, false};
static can_module_data can_module_interface_controler = {0x00000002, 0, false, false};

static struct can_module_data* can_modules[] = {
    &can_module_brodcast,
    &can_module_disrabution_controler,
    &can_module_interface_controler
};

const uint8_t can_paket_type_io_state = 0x01;
const uint8_t can_paket_type_adc_read = 0x02;
const uint8_t can_paket_type_status = 0x03;
const uint8_t can_paket_type_chanel_current = 0x04;
const uint8_t can_paket_type_chanel_set_curent_limit = 0x05;
const uint8_t can_paket_type_total_current = 0x06;
const uint8_t can_paket_type_chanel_control = 0x07;
const uint8_t can_paket_type_heartbeat = 0x08;
const uint8_t can_paket_type_chanel_fult = 0x09;
const uint8_t can_paket_type_module_fult = 0x0A;



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
void can_check_for_alerts (void *pvParameters);
struct message can_read_recived_frame();

#endif // can_h
